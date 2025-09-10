use crate::{EdgeData, Point};
use geo::{Distance, Haversine, InterpolatePoint, Point as GeoPoint};
use petgraph::graph::NodeIndex;
use std::collections::HashMap;

fn get_point_at_ratio(path: &[Point], ratio: f64) -> Option<Point> {
    if path.len() < 2 {
        return None;
    }

    let total_length = path
        .windows(2)
        .map(|p| Haversine.distance(GeoPoint::new(p[0].lon, p[0].lat), GeoPoint::new(p[1].lon, p[1].lat)))
        .sum::<f64>();

    if total_length == 0.0 {
        return path.first().copied();
    }

    let target_dist = total_length * ratio;
    let mut dist_traveled = 0.0;

    for p_window in path.windows(2) {
        let p1 = p_window[0];
        let p2 = p_window[1];
        let p1_geo = GeoPoint::new(p1.lon, p1.lat);
        let p2_geo = GeoPoint::new(p2.lon, p2.lat);

        let segment_len = Haversine.distance(p1_geo, p2_geo);

        if dist_traveled + segment_len >= target_dist {
            let needed_dist = target_dist - dist_traveled;
            let ratio_on_segment = if segment_len > 0.0 { needed_dist / segment_len } else { 0.0 };
            let intermediate_geo_point = Haversine.point_at_ratio_between(p1_geo, p2_geo, ratio_on_segment);
            return Some(Point {
                lat: intermediate_geo_point.y(),
                lon: intermediate_geo_point.x(),
            });
        }
        dist_traveled += segment_len;
    }

    path.last().copied() // Fallback to last point
}

// Helper function to offset a path for visualizing overlapping routes
fn offset_path(path: &[Point], pass_num: u32, total_passes: u32, offset_scale: f64) -> Vec<Point> {
    if total_passes <= 1 {
        return path.to_vec();
    }
    let shift = (pass_num as f64 - (total_passes as f64 - 1.0) / 2.0) * offset_scale;

    if path.len() < 2 {
        return path.to_vec();
    }

    path.windows(2).enumerate().flat_map(|(i, p_window)| {
        let p1 = p_window[0];
        let p2 = p_window[1];

        // Create a canonical representation of the segment to get a consistent perpendicular vector.
        // We can order by longitude, then latitude.
        let (u, v) = if (p1.lon, p1.lat) < (p2.lon, p2.lat) { (p1, p2) } else { (p2, p1) };
        let dx = v.lon - u.lon;
        let dy = v.lat - u.lat;

        let magnitude = (dx.powi(2) + dy.powi(2)).sqrt();
        let (offset_dx, offset_dy) = if magnitude > 0.0 {
            let perp_dx = -dy / magnitude;
            let perp_dy = dx / magnitude;
            (perp_dx * shift, perp_dy * shift)
        } else {
            (0.0, 0.0)
        };

        if i == 0 {
            vec![
                Point { lon: p1.lon + offset_dx, lat: p1.lat + offset_dy },
                Point { lon: p2.lon + offset_dx, lat: p2.lat + offset_dy },
            ]
        } else {
            vec![Point { lon: p2.lon + offset_dx, lat: p2.lat + offset_dy }]
        }
    }).collect()
}

pub fn export_route_map(routes: &[Vec<EdgeData>], candidate_segments: &[EdgeData], title: &str, offset_scale: f64) -> String {
    let colors = ["blue", "red", "green", "purple", "orange", "darkred", "lightred", "darkblue", "cadetblue"];

    let mut directed_counts = HashMap::<(NodeIndex, NodeIndex), u32>::new();
    for route in routes {
        for segment in route {
            *directed_counts.entry((segment.start_node, segment.end_node)).or_insert(0) += 1;
        }
    }

    let mut pass_num_tracker = HashMap::<(NodeIndex, NodeIndex), u32>::new();
    let mut layer_group_definitions = Vec::new();
    let mut overlay_map_entries = Vec::new();
    let mut all_layers_for_bounds = Vec::new();

    for (i, route) in routes.iter().enumerate() {
        let route_color = colors[i % colors.len()];
        let total_route_dist_km: f64 = route.iter().map(|e| e.distance).sum::<f64>() / 1000.0;
        let total_route_ascent_m: f64 = route.iter().map(|e| e.ascent).sum();
        let mut route_specific_layers = Vec::new();

        for segment in route {
            let u = segment.start_node;
            let v = segment.end_node;

            let forward_count = *directed_counts.get(&(u, v)).unwrap_or(&0);
            let reverse_count = *directed_counts.get(&(v, u)).unwrap_or(&0);
            let total_passes = forward_count + reverse_count;

            let current_pass_instance = *pass_num_tracker.entry((u, v)).or_insert(0);
            pass_num_tracker.insert((u, v), current_pass_instance + 1);

            let pass_num = if u.index() < v.index() { current_pass_instance } else { reverse_count + current_pass_instance };

            let offset_path_points = offset_path(&segment.path, pass_num, total_passes, offset_scale);
            let js_points: Vec<String> = offset_path_points.iter().map(|p| format!("[{}, {}]", p.lat, p.lon)).collect();
            let js_coordinates = format!("[{}]", js_points.join(", "));

            let popup_content = format!(
                "<b>Route #{}</b><br>Total Dist: {:.2}km<br>Total Ascent: {:.1}m<hr><b>Segment #{}</b><br>Dist: {:.1}m<br>Ascent: {:.1}m",
                i + 1, total_route_dist_km, total_route_ascent_m, segment.segment_id, segment.distance, segment.ascent
            ).replace("'", "\\'");

            let polyline_str = format!(
                "L.polyline({js_coordinates}, {{ color: '{color}', weight: 3 }}).bindPopup('{popup_content}')",
                js_coordinates = js_coordinates, color = route_color, popup_content = popup_content
            );

            route_specific_layers.push(polyline_str.clone());
            all_layers_for_bounds.push(polyline_str);

            if let Some(marker_point) = get_point_at_ratio(&segment.path, 0.75) {
                 let marker_str = format!(
                    "L.circleMarker([{}, {}], {{ radius: 2, color: '{color}', fillColor: '{color}', fillOpacity: 1.0 }})",
                    marker_point.lat, marker_point.lon, color = route_color
                );
                 route_specific_layers.push(marker_str.clone());
                 all_layers_for_bounds.push(marker_str);
            }
        }

        let route_group_var = format!("route{}Group", i);
        layer_group_definitions.push(format!("var {} = L.layerGroup([{}]);", route_group_var, route_specific_layers.join(", ")));

        let overlay_label = format!("Route #{} ({:.2} km, {:.0}m ascent)", i + 1, total_route_dist_km, total_route_ascent_m);
        overlay_map_entries.push(format!("\"{}\": {}", overlay_label.replace("'", "\\'"), route_group_var));
    }

    // --- Add Ascent Segments Layer ---
    let mut ascent_segment_layers = Vec::new();
    for (i, segment) in candidate_segments.iter().enumerate() {
        // We don't want to offset these, as they are the base segments.
        let js_points: Vec<String> = segment.path.iter().map(|p| format!("[{}, {}]", p.lat, p.lon)).collect();
        let js_coordinates = format!("[{}]", js_points.join(", "));

        let popup_content = format!(
            "<b>Ascent Segment Rank: #{}</b><br>Distance: {:.1}m<br>Ascent: {:.1}m",
            i + 1, segment.distance, segment.ascent
        ).replace("'", "\\'");

        let polyline_str = format!(
            "L.polyline({js_coordinates}, {{ color: 'black', weight: 4, dashArray: '5, 5' }}).bindPopup('{popup_content}')",
            js_coordinates = js_coordinates, popup_content = popup_content
        );

        ascent_segment_layers.push(polyline_str.clone());
        all_layers_for_bounds.push(polyline_str);

        if let Some(marker_point) = get_point_at_ratio(&segment.path, 0.75) {
             let marker_str = format!(
                "L.circleMarker([{}, {}], {{ radius: 3, color: 'black', fillColor: 'black', fillOpacity: 1.0 }})",
                marker_point.lat, marker_point.lon
            );
             ascent_segment_layers.push(marker_str.clone());
             all_layers_for_bounds.push(marker_str);
        }
    }

    if !ascent_segment_layers.is_empty() {
        let ascent_group_var = "ascentSegmentsGroup";
        layer_group_definitions.push(format!("var {} = L.layerGroup([{}]);", ascent_group_var, ascent_segment_layers.join(", ")));
        overlay_map_entries.push(format!("\"Top Ascent Segments\": {}", ascent_group_var));
    }

    // --- Ascent Segment Layer ---
    let mut ascent_layer_definition = String::new();
    let ascent_group_var_name = if !ascent_segment_layers.is_empty() {
        let var_name = "ascentSegmentsGroup";
        ascent_layer_definition = format!("var {} = L.layerGroup([{}]);", var_name, ascent_segment_layers.join(", "));
        var_name
    } else {
        "null"
    };

    let script_logic = format!(
        r#"
    var map = L.map('map');

    var standard = L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }});

    var opentopo = L.tileLayer('https://{{s}}.tile.opentopomap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: 'Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, SRTM | Map style: &copy; <a href="https://opentopomap.org">OpenTopoMap</a> (CC-BY-SA)'
    }});

    var cyclosm = L.tileLayer('https://{{s}}.tile-cyclosm.openstreetmap.fr/cyclosm/{{z}}/{{x}}/{{y}}.png', {{
        attribution: '<a href="https://github.com/cyclosm/cyclosm-cartocss-style/releases" title="CyclOSM - Open Bicycle render">CyclOSM</a> | Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }});

    cyclosm.addTo(map);

    var baseMaps = {{
        "Standard": standard,
        "OpenTopoMap": opentopo,
        "CyclOSM": cyclosm
    }};

    {layer_group_definitions}
    {ascent_layer_definition}

    var overlayMaps = {{
        {overlay_map_entries}
    }};

    var allLayers = [{all_layers_for_bounds}];
    var boundsGroup = L.featureGroup(allLayers);

    if (boundsGroup.getLayers().length > 0) {{
        map.fitBounds(boundsGroup.getBounds().pad(0.1));
    }} else {{
        map.setView([51.505, -0.09], 13);
    }}

    var layerControl = L.control.layers(baseMaps, overlayMaps).addTo(map);

    // --- Custom Controls Logic ---
    var ascentLayer = {ascent_group_var_name};

    if (ascentLayer) {{
        var checkboxContainer = document.getElementById('overlay-checkboxes');
        var checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = 'ascent-checkbox';
        checkbox.checked = true;
        checkboxContainer.appendChild(checkbox);

        var label = document.createElement('label');
        label.htmlFor = 'ascent-checkbox';
        label.textContent = 'Top Ascent Segments';
        checkboxContainer.appendChild(label);

        map.addLayer(ascentLayer);

        checkbox.addEventListener('change', function() {{
            if (this.checked) {{
                map.addLayer(ascentLayer);
            }} else {{
                map.removeLayer(ascentLayer);
            }}
        }});
    }}

    var overlayKeys = Object.keys(overlayMaps);

    // 1. Bulk action buttons
    document.getElementById('show-all-btn').addEventListener('click', function() {{
        overlayKeys.forEach(function(key) {{
            map.addLayer(overlayMaps[key]);
        }});
    }});

    document.getElementById('hide-all-btn').addEventListener('click', function() {{
        overlayKeys.forEach(function(key) {{
            map.removeLayer(overlayMaps[key]);
        }});
    }});

    // 2. Radio button logic
    var radioContainer = document.getElementById('route-radio-buttons');

    // "None" option
    var noneRadio = document.createElement('input');
    noneRadio.type = 'radio';
    noneRadio.id = 'radio-none';
    noneRadio.name = 'route-selector';
    noneRadio.checked = true;
    radioContainer.appendChild(noneRadio);
    var noneLabel = document.createElement('label');
    noneLabel.htmlFor = 'radio-none';
    noneLabel.textContent = 'None';
    radioContainer.appendChild(noneLabel);
    radioContainer.appendChild(document.createElement('br'));

    noneRadio.addEventListener('change', function() {{
        if (this.checked) {{
            overlayKeys.forEach(function(key) {{
                map.removeLayer(overlayMaps[key]);
            }});
        }}
    }});

    // Route-specific options
    overlayKeys.forEach(function(key, i) {{
        var radio = document.createElement('input');
        radio.type = 'radio';
        radio.id = 'radio-' + i;
        radio.name = 'route-selector';
        radio.value = key;
        radioContainer.appendChild(radio);

        var label = document.createElement('label');
        label.htmlFor = 'radio-' + i;
        label.textContent = key;
        radioContainer.appendChild(label);
        radioContainer.appendChild(document.createElement('br'));

        radio.addEventListener('change', function() {{
            if (this.checked) {{
                overlayKeys.forEach(function(otherKey) {{
                    if (otherKey !== key) {{
                        map.removeLayer(overlayMaps[otherKey]);
                    }}
                }});
                map.addLayer(overlayMaps[key]);
            }}
        }});
    }});


    // Hide the original checkbox-based layer control
    layerControl.getContainer().style.display = 'none';

    // Set initial state: show all layers by default
    document.getElementById('show-all-btn').click();
"#,
        layer_group_definitions = layer_group_definitions.join("\n    "),
        ascent_layer_definition = ascent_layer_definition,
        ascent_group_var_name = ascent_group_var_name,
        overlay_map_entries = overlay_map_entries.join(",\n        "),
        all_layers_for_bounds = all_layers_for_bounds.join(",\n")
    );

    format!(
        r#"
<!DOCTYPE html>
<html>
<head>
    <title>{title}</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <style>
        html, body {{ height: 100%; margin: 0; }}
        #map {{ width: 100%; height: 100%; }}
        .custom-control-container {{
            position: absolute;
            top: 10px;
            right: 10px;
            z-index: 1000;
            background: white;
            padding: 10px;
            border: 1px solid #ccc;
            border-radius: 5px;
            max-width: 280px;
        }}
        .custom-control-container .control-row {{
            margin-bottom: 8px;
        }}
        .custom-control-container button {{
            margin-right: 5px;
        }}
        .custom-control-container label {{
            font-weight: bold;
            display: block;
            margin-bottom: 4px;
        }}
        #route-radio-buttons label, #overlay-checkboxes label {{
            font-weight: normal;
            display: inline-block;
            margin-left: 4px;
            margin-right: 10px;
        }}
        hr {{
            margin-top: 10px;
            margin-bottom: 10px;
            border: 0;
            border-top: 1px solid #ccc;
        }}
    </style>
</head>
<body>

<div id="map"></div>
<div class="custom-control-container leaflet-bar">
    <div class="control-row">
        <label>Overlays</label>
        <div id="overlay-checkboxes"></div>
    </div>
    <hr>
    <div class="control-row">
        <label>Select Route</label>
        <div id="route-radio-buttons"></div>
    </div>
    <hr>
    <div class="control-row">
        <label>Bulk Actions (Routes)</label>
        <button id="show-all-btn">Show All</button>
        <button id="hide-all-btn">Hide All</button>
    </div>
</div>

<script>
{script_logic}
</script>
</body>
</html>
"#,
        title = title,
        script_logic = script_logic
    )
}
