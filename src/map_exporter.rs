use crate::{EdgeData, Point};

pub fn export_combined_map(
    top_edges: &[EdgeData],
    parking_locations: &[Point],
    title: &str,
) -> String {
    let edge_polylines: Vec<String> = top_edges
        .iter()
        .enumerate()
        .map(|(i, edge)| {
            let coordinates: Vec<String> = edge
                .path
                .iter()
                .map(|p| format!("[{}, {}]", p.lat, p.lon))
                .collect();
            let js_coordinates = format!("[{}]", coordinates.join(", "));
            let color = "red";
            let popup_content = format!(
                "<b>Edge #{}</b><br>Distance: {:.2}m<br>Ascent: {:.2}m<br>Descent: {:.2}m",
                i + 1,
                edge.distance,
                edge.ascent,
                edge.descent
            );

            format!(
                "L.polyline({js_coordinates}, {{ color: '{color}' }}).bindPopup('{popup_content}')",
                js_coordinates = js_coordinates,
                color = color,
                popup_content = popup_content
            )
        })
        .collect();

    let parking_markers: Vec<String> = parking_locations
        .iter()
        .map(|p| format!("L.marker([{}, {}]).bindPopup('Parking')", p.lat, p.lon))
        .collect();

    let all_layers = [edge_polylines, parking_markers].concat();

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
    </style>
</head>
<body>

<div id="map"></div>

<script>
    var map = L.map('map');

    var osm = L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }});

    var opentopo = L.tileLayer('https://{{s}}.tile.opentopomap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: 'Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, SRTM | Map style: &copy; <a href="https://opentopomap.org">OpenTopoMap</a> (CC-BY-SA)'
    }});

    opentopo.addTo(map);

    var baseMaps = {{
        "OpenStreetMap": osm,
        "OpenTopoMap": opentopo
    }};

    L.control.layers(baseMaps).addTo(map);

    var layers = [
        {layers_script}
    ];

    var featureGroup = L.featureGroup(layers).addTo(map);

    if (layers.length > 0) {{
        map.fitBounds(featureGroup.getBounds().pad(0.1));
    }} else {{
        map.setView([51.505, -0.09], 13);
    }}
</script>

</body>
</html>
"#,
        title = title,
        layers_script = all_layers.join(",\n")
    )
}

pub fn export_route_map(route: &[EdgeData], title: &str) -> String {
    let all_points: Vec<String> = route
        .iter()
        .flat_map(|edge| edge.path.iter())
        .map(|p| format!("[{}, {}]", p.lat, p.lon))
        .collect();

    let js_coordinates = format!("[{}]", all_points.join(", "));
    let route_polyline = format!("L.polyline({js_coordinates}, {{ color: 'blue' }})");

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
    </style>
</head>
<body>
<div id="map"></div>
<script>
    var map = L.map('map');
    var opentopo = L.tileLayer('https://{{s}}.tile.opentopomap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: 'Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, SRTM | Map style: &copy; <a href="https://opentopomap.org">OpenTopoMap</a> (CC-BY-SA)'
    }}).addTo(map);

    var routeLayer = {route_polyline};
    routeLayer.addTo(map);
    map.fitBounds(routeLayer.getBounds().pad(0.1));
</script>
</body>
</html>
"#,
        title = title,
        route_polyline = route_polyline
    )
}
