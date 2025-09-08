use geo::{HaversineDistance, HaversineIntermediate, Point as GeoPoint};
use ndarray::Array2;
use osmpbf::{Element, ElementReader};
use petgraph::graph::Graph;
use std::collections::HashMap;
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::Read;
use tiff::decoder::{Decoder, DecodingResult};

use tiff::tags::Tag;

pub mod map_exporter;

#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub lat: f64,
    pub lon: f64,
}

#[derive(Debug, Clone)]
pub struct EdgeData {
    pub path: Vec<Point>,
    pub distance: f64,
    pub ascent: f64,
    pub descent: f64,
}

fn is_valid_way<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> bool {
    let mut highway_val: Option<&str> = None;
    let mut access_val: Option<&str> = None;

    for (key, value) in tags {
        if key == "highway" {
            highway_val = Some(value);
        } else if key == "access" {
            access_val = Some(value);
        }
    }

    if let Some(access) = access_val {
        if matches!(access, "private" | "no") {
            return false;
        }
    }

    if let Some(highway) = highway_val {
        return match highway {
            "path" | "footway" | "track" | "bridleway" | "cycleway" | "residential"
            | "unclassified" | "tertiary" => true,
            "motorway" | "primary" | "trunk" => false,
            _ => false, // Default to not including other highway types
        };
    }

    false
}

fn centroid(points: &[Point]) -> Option<Point> {
    if points.is_empty() {
        return None;
    }
    let mut lat_sum = 0.0;
    let mut lon_sum = 0.0;
    for point in points {
        lat_sum += point.lat;
        lon_sum += point.lon;
    }
    let count = points.len() as f64;
    Some(Point {
        lat: lat_sum / count,
        lon: lon_sum / count,
    })
}

fn get_geotransform(path: &str) -> Result<[f64; 6], Box<dyn Error>> {
    let mut file = File::open(path)?;
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer)?;

    let mut decoder = Decoder::new(std::io::Cursor::new(buffer))?;

    let tiepoint = decoder.get_tag_f64_vec(Tag::ModelTiepointTag)?;
    let pixel_scale = decoder.get_tag_f64_vec(Tag::ModelPixelScaleTag)?;

    // Assuming the first tiepoint is the origin
    let i = tiepoint[0];
    let j = tiepoint[1];
    let _k = tiepoint[2];
    let x = tiepoint[3];
    let y = tiepoint[4];
    let _z = tiepoint[5];

    let sx = pixel_scale[0];
    let sy = pixel_scale[1];

    // Construct the geotransform array
    // [top_left_x, x_resolution, 0.0, top_left_y, 0.0, -y_resolution]
    Ok([x - (i * sx), sx, 0.0, y + (j * sy), 0.0, -sy])
}

fn get_interpolated_elevation(
    point: &Point,
    elevation_data: &Array2<i16>,
    geo_transform: &[f64; 6],
) -> Option<f64> {
    let col = ((point.lon - geo_transform[0]) / geo_transform[1]).floor();
    let row = ((point.lat - geo_transform[3]) / geo_transform[5]).floor();

    let (height, width) = (elevation_data.shape()[0], elevation_data.shape()[1]);

    if row < 0.0 || col < 0.0 || row + 1.0 >= height as f64 || col + 1.0 >= width as f64 {
        return None; // Out of bounds
    }

    let x_frac = ((point.lon - geo_transform[0]) / geo_transform[1]) - col;
    let y_frac = ((point.lat - geo_transform[3]) / geo_transform[5]) - row;

    let q11 = *elevation_data.get((row as usize, col as usize))? as f64;
    let q21 = *elevation_data.get((row as usize, col as usize + 1))? as f64;
    let q12 = *elevation_data.get((row as usize + 1, col as usize))? as f64;
    let q22 = *elevation_data.get((row as usize + 1, col as usize + 1))? as f64;

    let r1 = q11 * (1.0 - x_frac) + q21 * x_frac;
    let r2 = q12 * (1.0 - x_frac) + q22 * x_frac;

    Some(r1 * (1.0 - y_frac) + r2 * y_frac)
}

fn main() -> Result<(), Box<dyn Error>> {
    let osm_path = "data/oxfordshire-250907.osm.pbf";
    let srtm_path = "data/oxfordshire_ish.SRTMGL1.tif";
    const INTERPOLATION_DISTANCE_M: f64 = 10.0;

    // --- Pass 1: Caching node coordinates ---
    println!("Starting pass 1: Caching node coordinates...");
    let mut node_coords = HashMap::new();
    let reader1 = ElementReader::from_path(osm_path)?;
    reader1.for_each(|element| {
        if let Element::Node(node) = element {
            node_coords.insert(
                node.id(),
                Point {
                    lat: node.lat(),
                    lon: node.lon(),
                },
            );
        } else if let Element::DenseNode(node) = element {
            node_coords.insert(
                node.id(),
                Point {
                    lat: node.lat(),
                    lon: node.lon(),
                },
            );
        }
    })?;
    println!("Pass 1 complete. Found {} nodes.", node_coords.len());

    // --- Pass 2: Find parking amenities ---
    println!("Starting pass 2: Finding parking locations...");
    let reader2 = ElementReader::from_path(osm_path)?;
    let mut parking_locations = Vec::new();
    let mut parking_nodes = 0;
    let mut parking_ways = 0;

    reader2.for_each(|element| match element {
        Element::Node(node) => {
            if node
                .tags()
                .any(|(key, value)| key == "amenity" && value == "parking")
            {
                parking_locations.push(Point {
                    lat: node.lat(),
                    lon: node.lon(),
                });
                parking_nodes += 1;
            }
        }
        Element::DenseNode(node) => {
            if node
                .tags()
                .any(|(key, value)| key == "amenity" && value == "parking")
            {
                parking_locations.push(Point {
                    lat: node.lat(),
                    lon: node.lon(),
                });
                parking_nodes += 1;
            }
        }
        Element::Way(way) => {
            if way
                .tags()
                .any(|(key, value)| key == "amenity" && value == "parking")
            {
                let way_points: Vec<Point> = way
                    .refs()
                    .filter_map(|node_id| node_coords.get(&node_id).copied())
                    .collect();
                if let Some(center) = centroid(&way_points) {
                    parking_locations.push(center);
                    parking_ways += 1;
                }
            }
        }
        _ => (),
    })?;
    println!(
        "Pass 2 complete. Found {} parking locations ({} from nodes, {} from ways).",
        parking_locations.len(),
        parking_nodes,
        parking_ways
    );

    // --- Pass 3: Count node references to find intersections ---
    println!("Starting pass 3: Identifying intersections...");
    let mut node_ref_counts = HashMap::new();
    let reader3 = ElementReader::from_path(osm_path)?;
    reader3.for_each(|element| {
        if let Element::Way(way) = element {
            if is_valid_way(way.tags()) {
                for node_id in way.refs() {
                    *node_ref_counts.entry(node_id).or_insert(0) += 1;
                }
            }
        }
    })?;
    let intersection_nodes: HashMap<_, _> = node_ref_counts
        .into_iter()
        .filter(|&(_, count)| count > 1)
        .collect();
    println!(
        "Pass 3 complete. Found {} intersection nodes.",
        intersection_nodes.len()
    );

    // --- Pass 4: Build the graph ---
    println!("Starting pass 4: Building graph...");
    let mut graph = Graph::<Point, EdgeData>::new();
    let mut osm_id_to_node_index = HashMap::new();

    for (osm_id, _) in &intersection_nodes {
        if let Some(point) = node_coords.get(osm_id) {
            let node_index = graph.add_node(*point);
            osm_id_to_node_index.insert(*osm_id, node_index);
        }
    }

    let reader4 = ElementReader::from_path(osm_path)?;
    reader4.for_each(|element| {
        if let Element::Way(way) = element {
            if is_valid_way(way.tags()) {
                let mut last_intersection_node_id: Option<i64> = None;
                let mut current_path_segment = Vec::new();
                for node_id in way.refs() {
                    let point = node_coords.get(&node_id).unwrap();
                    current_path_segment.push(*point);
                    if intersection_nodes.contains_key(&node_id) {
                        if let Some(last_id) = last_intersection_node_id {
                            let start_node_idx = osm_id_to_node_index.get(&last_id).unwrap();
                            let end_node_idx = osm_id_to_node_index.get(&node_id).unwrap();
                            graph.add_edge(
                                *start_node_idx,
                                *end_node_idx,
                                EdgeData {
                                    path: current_path_segment.clone(),
                                    distance: 0.0,
                                    ascent: 0.0,
                                    descent: 0.0,
                                },
                            );
                        }
                        last_intersection_node_id = Some(node_id);
                        current_path_segment = vec![*point];
                    }
                }
            }
        }
    })?;
    println!(
        "Pass 4 complete. Graph has {} nodes and {} edges.",
        graph.node_count(),
        graph.edge_count()
    );

    // --- Pass 5: Calculate edge weights with interpolation ---
    println!("Starting pass 5: Calculating edge weights...");
    let geo_transform = get_geotransform(srtm_path)?;
    let mut file = File::open(srtm_path)?;
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer)?;
    let mut decoder = Decoder::new(std::io::Cursor::new(buffer))?;
    let (width, height) = decoder.dimensions()?;
    let image_data = match decoder.read_image()? {
        DecodingResult::I16(data) => data,
        _ => return Err("Unsupported TIFF data type".into()),
    };
    let elevation_data = Array2::from_shape_vec((height as usize, width as usize), image_data)?;

    let mut total_distance = 0.0;
    let mut total_ascent = 0.0;

    for edge in graph.edge_weights_mut() {
        let mut edge_distance = 0.0;
        let mut edge_ascent = 0.0;
        let mut edge_descent = 0.0;
        let mut last_elevation: Option<f64> = None;

        for points in edge.path.windows(2) {
            let p1 = points[0];
            let p2 = points[1];
            let geo_p1 = GeoPoint::new(p1.lon, p1.lat);
            let geo_p2 = GeoPoint::new(p2.lon, p2.lat);
            let segment_distance = geo_p1.haversine_distance(&geo_p2);
            edge_distance += segment_distance;

            let num_steps = (segment_distance / INTERPOLATION_DISTANCE_M).ceil() as usize;
            if num_steps == 0 {
                continue;
            }

            for i in 0..=num_steps {
                let fraction = if num_steps > 0 {
                    i as f64 / num_steps as f64
                } else {
                    0.0
                };
                let intermediate_geo_point = geo_p1.haversine_intermediate(&geo_p2, fraction);
                let intermediate_point = Point {
                    lat: intermediate_geo_point.y(),
                    lon: intermediate_geo_point.x(),
                };

                if let Some(elevation) =
                    get_interpolated_elevation(&intermediate_point, &elevation_data, &geo_transform)
                {
                    if let Some(last_elev) = last_elevation {
                        let delta = elevation - last_elev;
                        if delta > 0.0 {
                            edge_ascent += delta;
                        } else {
                            edge_descent -= delta;
                        }
                    }
                    last_elevation = Some(elevation);
                }
            }
        }

        edge.distance = edge_distance;
        edge.ascent = edge_ascent;
        edge.descent = edge_descent;

        total_distance += edge_distance;
        total_ascent += edge_ascent;
    }

    println!("Pass 5 complete.");
    println!("Total distance: {}, total ascent: {}", total_distance, total_ascent);

    println!("\n--- Top Edges by Ascent ---");
    let mut top_edges: Vec<_> = graph.edge_weights().cloned().collect();
    top_edges.sort_by(|a, b| {
        let a_ascent = f64::max(a.ascent, a.descent);
        let b_ascent = f64::max(b.ascent, b.descent);
        b_ascent
            .partial_cmp(&a_ascent)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let top_edges: Vec<_> = top_edges.iter().take(50).cloned().collect();

    println!(
        "{:<10} | {:<10} | {:<12} | {:<25}",
        "Ascent (m)", "Descent (m)", "Distance (m)", "Centroid (Lat, Lon)"
    );
    println!("{:-<11}|{:-<12}|{:-<14}|{:-<26}", "", "", "", "");
    for edge in &top_edges {
        if let Some(center) = centroid(&edge.path) {
            println!(
                "{:<10.2} | {:<10.2} | {:<12.2} | ({:.6}, {:.6})",
                edge.ascent, edge.descent, edge.distance, center.lat, center.lon
            );
        }
    }

    parking_locations.clear();

    // --- Pass 6: Generate combined map ---
    println!("\n--- Generating Combined Map ---");
    fs::create_dir_all("vis")?;
    let map_title = "Top Steepest Edges and Parking";
    let html_content = map_exporter::export_combined_map(&top_edges, &parking_locations, map_title);
    let filename = "vis/map.html";
    fs::write(filename, html_content)?;
    println!("-> Saved combined map to {}", filename);

    println!("\n--- Network Summary ---");
    println!("Total Network Distance: {:.2} km", total_distance / 1000.0);
    println!("Total Network Ascent: {:.2} m", total_ascent);

    Ok(())
}
