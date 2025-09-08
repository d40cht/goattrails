use crate::Point;

pub fn export_to_html(path: &[Point]) -> String {
    let coordinates: Vec<String> = path
        .iter()
        .map(|p| format!("[{}, {}]", p.lat, p.lon))
        .collect();

    let js_coordinates = format!("[{}]", coordinates.join(", "));

    format!(
        r#"
<!DOCTYPE html>
<html>
<head>
    <title>GoatTrails - Route Map</title>
    <meta charset="utf-g">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"
        integrity="sha512-xodZBNTC5n17Xt2atTPuE1HxjVMSvLVW9ocqUKLsCC5CXdbqCmblAshOMAS6/keqq/sMZMZ19scR4PsZChSR7A=="
        crossorigin=""/>
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"
        integrity="sha512-XQoYMqMTK8LvdxXYG3nZ448hOEQiglfqkJs1NOQV44cWnUrBc8PkAOcXy20w0vlaXaVUearIOBhiXZ5V3ynxwA=="
        crossorigin=""></script>
    <style>
        html, body {{
            height: 100%;
            margin: 0;
        }}
        #map {{
            width: 100%;
            height: 100%;
        }}
    </style>
</head>
<body>

<div id="map"></div>

<script>
    var map = L.map('map');

    L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }}).addTo(map);

    var routeCoords = {js_coordinates};

    if (routeCoords.length > 0) {{
        var polyline = L.polyline(routeCoords, {{color: 'red'}}).addTo(map);
        map.fitBounds(polyline.getBounds());
    }} else {{
        // Default view if no coordinates
        map.setView([51.505, -0.09], 13);
    }}
</script>

</body>
</html>
"#,
        js_coordinates = js_coordinates
    )
}
