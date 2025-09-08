use geo::{HaversineDistance, HaversineIntermediate, Point as GeoPoint};
use ndarray::Array2;
use osmpbf::{Element, ElementReader};
use petgraph::graph::{Graph, NodeIndex};
use std::collections::HashMap;
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::Read;
use tiff::decoder::{Decoder, DecodingResult};
use tiff::tags::Tag;
use petgraph::prelude::*;
use rand::seq::SliceRandom;
use serde::Deserialize;

pub mod map_exporter;
pub mod route_generator;

#[derive(Deserialize)]
struct Config {
    target_distance_km: f64,
    algorithm_iterations: usize,
    route_candidates_to_generate: usize,
    top_routes_to_display: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub lat: f64,
    pub lon: f64,
}

#[derive(Debug, Clone)]
pub struct EdgeData {
    pub original_way_id: i64,
    pub path: Vec<Point>,
    pub distance: f64,
    pub ascent: f64,
    pub descent: f64,
}

type RouteGraph = Graph<Point, EdgeData, petgraph::Directed>;

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
            _ => false,
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
    Some(Point { lat: lat_sum / count, lon: lon_sum / count })
}

fn get_geotransform(path: &str) -> Result<[f64; 6], Box<dyn Error>> {
    let mut file = File::open(path)?;
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer)?;

    let mut decoder = Decoder::new(std::io::Cursor::new(buffer))?;

    let tiepoint = decoder.get_tag_f64_vec(Tag::ModelTiepointTag)?;
    let pixel_scale = decoder.get_tag_f64_vec(Tag::ModelPixelScaleTag)?;

    let i = tiepoint[0];
    let j = tiepoint[1];
    let x = tiepoint[3];
    let y = tiepoint[4];
    let sx = pixel_scale[0];
    let sy = pixel_scale[1];

    Ok([x - (i * sx), sx, 0.0, y + (j * sy), 0.0, -sy])
}

fn get_interpolated_elevation(point: &Point, elevation_data: &Array2<i16>, geo_transform: &[f64; 6]) -> Option<f64> {
    let col = ((point.lon - geo_transform[0]) / geo_transform[1]).floor();
    let row = ((point.lat - geo_transform[3]) / geo_transform[5]).floor();

    let (height, width) = (elevation_data.shape()[0], elevation_data.shape()[1]);

    if row < 0.0 || col < 0.0 || row + 1.0 >= height as f64 || col + 1.0 >= width as f64 {
        return None;
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

fn build_graph(osm_path: &str, srtm_path: &str) -> Result<(RouteGraph, Vec<Point>), Box<dyn Error>> {
    const INTERPOLATION_DISTANCE_M: f64 = 10.0;
    println!("Building graph...");

    // Pass 1: Cache all node coordinates from the OSM file.
    let mut node_coords = HashMap::new();
    let reader1 = ElementReader::from_path(osm_path)?;
    reader1.for_each(|element| {
        if let Element::Node(node) = element {
            node_coords.insert(node.id(), Point { lat: node.lat(), lon: node.lon() });
        } else if let Element::DenseNode(node) = element {
            node_coords.insert(node.id(), Point { lat: node.lat(), lon: node.lon() });
        }
    })?;

    // Pass 2: Find parking amenities to use as potential start/end points.
    let mut parking_locations = Vec::new();
    let reader2 = ElementReader::from_path(osm_path)?;
    reader2.for_each(|element| {
        match element {
            Element::Node(n) if n.tags().any(|(k, v)| k == "amenity" && v == "parking") => {
                parking_locations.push(Point { lat: n.lat(), lon: n.lon() });
            },
            Element::DenseNode(n) if n.tags().any(|(k, v)| k == "amenity" && v == "parking") => {
                parking_locations.push(Point { lat: n.lat(), lon: n.lon() });
            },
            Element::Way(w) if w.tags().any(|(k, v)| k == "amenity" && v == "parking") => {
                let way_points: Vec<Point> = w.refs().filter_map(|id| node_coords.get(&id).copied()).collect();
                if let Some(center) = centroid(&way_points) {
                    parking_locations.push(center);
                }
            },
            _ => (),
        }
    })?;

    // Pass 3: Count node references in valid ways to identify intersections.
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
    let intersection_nodes: HashMap<_,_> = node_ref_counts.into_iter().filter(|&(_, count)| count > 1).collect();

    // Pass 4: Build the graph structure.
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
                            let start_idx = *osm_id_to_node_index.get(&last_id).unwrap();
                            let end_idx = *osm_id_to_node_index.get(&node_id).unwrap();
                            graph.add_edge(start_idx, end_idx, EdgeData { original_way_id: way.id(), path: current_path_segment.clone(), distance: 0.0, ascent: 0.0, descent: 0.0 });
                        }
                        last_intersection_node_id = Some(node_id);
                        current_path_segment = vec![*point];
                    }
                }
            }
        }
    })?;

    // Pass 5: Calculate edge weights (distance, ascent, descent) using SRTM data.
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

    for edge in graph.edge_weights_mut() {
        let (mut distance, mut ascent, mut descent) = (0.0, 0.0, 0.0);
        let mut last_elevation: Option<f64> = None;

        for points in edge.path.windows(2) {
            let p1 = GeoPoint::new(points[0].lon, points[0].lat);
            let p2 = GeoPoint::new(points[1].lon, points[1].lat);
            let segment_distance = p1.haversine_distance(&p2);
            distance += segment_distance;

            let num_steps = (segment_distance / INTERPOLATION_DISTANCE_M).ceil() as usize;
            if num_steps == 0 { continue; }

            for i in 0..=num_steps {
                let fraction = if num_steps > 0 { i as f64 / num_steps as f64 } else { 0.0 };
                let intermediate_geo_point = p1.haversine_intermediate(&p2, fraction);
                let intermediate_point = Point { lat: intermediate_geo_point.y(), lon: intermediate_geo_point.x() };

                if let Some(elevation) = get_interpolated_elevation(&intermediate_point, &elevation_data, &geo_transform) {
                    if let Some(last_elev) = last_elevation {
                        let delta = elevation - last_elev;
                        if delta > 0.0 {
                            ascent += delta;
                        } else {
                            descent -= delta;
                        }
                    }
                    last_elevation = Some(elevation);
                }
            }
        }
        edge.distance = distance;
        edge.ascent = ascent;
        edge.descent = descent;
    }

    println!("Adding reverse edges...");
    let mut reverse_edges = Vec::new();
    for edge_ref in graph.edge_references() {
        let source = edge_ref.source();
        let target = edge_ref.target();
        let weight = edge_ref.weight();

        let mut reversed_path = weight.path.clone();
        reversed_path.reverse();

        reverse_edges.push((target, source, EdgeData {
            original_way_id: weight.original_way_id,
            path: reversed_path,
            distance: weight.distance,
            ascent: weight.descent,
            descent: weight.ascent,
        }));
    }

    for (source, target, data) in reverse_edges {
        graph.add_edge(source, target, data);
    }

    println!("Graph build complete. Final graph has {} nodes and {} edges.", graph.node_count(), graph.edge_count());
    Ok((graph, parking_locations))
}

fn find_nearest_node(graph: &RouteGraph, point: &Point) -> Option<NodeIndex> {
    graph.node_indices().min_by(|&a, &b| {
        let p_a = graph[a];
        let p_b = graph[b];
        let dist_a = GeoPoint::new(p_a.lon, p_a.lat).haversine_distance(&GeoPoint::new(point.lon, point.lat));
        let dist_b = GeoPoint::new(p_b.lon, p_b.lat).haversine_distance(&GeoPoint::new(point.lon, point.lat));
        dist_a.partial_cmp(&dist_b).unwrap()
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let config_str = fs::read_to_string("config.toml")?;
    let config: Config = toml::from_str(&config_str)?;

    let osm_path = "data/oxfordshire-250907.osm.pbf";
    let srtm_path = "data/oxfordshire_ish.SRTMGL1.tif";

    let (graph, parking_locations) = build_graph(osm_path, srtm_path)?;

    if parking_locations.is_empty() {
        eprintln!("No parking locations found. Cannot generate a route.");
        return Ok(());
    }

    let mut rng = rand::thread_rng();
    let random_parking_spot = parking_locations.choose(&mut rng).unwrap();

    println!("\nSelected random starting point near: ({:.4}, {:.4})", random_parking_spot.lat, random_parking_spot.lon);

    if let Some(start_node) = find_nearest_node(&graph, random_parking_spot) {
        println!("Found nearest graph node to start route generation.");

        let mut generated_routes = Vec::new();
        for i in 0..config.route_candidates_to_generate {
            println!("\n--- Generating Route Candidate {}/{} ---", i + 1, config.route_candidates_to_generate);
            if let Some(route) = route_generator::generate_route(&graph, start_node, config.target_distance_km * 1000.0, config.algorithm_iterations) {
                generated_routes.push(route);
            } else {
                eprintln!("Failed to generate a route candidate after {} iterations.", config.algorithm_iterations);
            }
        }

        if generated_routes.is_empty() {
            eprintln!("No routes were generated.");
            return Ok(());
        }

        println!("\n--- All routes generated. Ranking by ascent... ---");

        generated_routes.sort_by(|a, b| {
            let a_ascent: f64 = a.iter().map(|e| e.ascent).sum();
            let b_ascent: f64 = b.iter().map(|e| e.ascent).sum();
            b_ascent.partial_cmp(&a_ascent).unwrap()
        });

        let top_routes: Vec<_> = generated_routes.into_iter().take(config.top_routes_to_display).collect();

        println!("\n--- Top {} Routes ---", top_routes.len());
        for (i, route) in top_routes.iter().enumerate() {
            let total_dist: f64 = route.iter().map(|e| e.distance).sum();
            let total_ascent: f64 = route.iter().map(|e| e.ascent).sum();
            println!("\nRoute #{}: Distance = {:.2}km, Ascent = {:.2}m", i + 1, total_dist / 1000.0, total_ascent);
            println!("{:<15} | {:<12} | {:<10} | {:<10}", "Way ID", "Distance (m)", "Ascent (m)", "Descent (m)");
            println!("{:-<16}|{:-<14}|{:-<12}|{:-<12}", "", "", "", "");
            for segment in route {
                println!("{:<15} | {:<12.2} | {:<10.2} | {:<10.2}", segment.original_way_id, segment.distance, segment.ascent, segment.descent);
            }
        }

        fs::create_dir_all("vis")?;
        let map_title = "Top Generated Routes";
        let html_content = map_exporter::export_route_map(&top_routes, map_title);
        let filename = "vis/final_route.html";
        fs::write(filename, html_content)?;
        println!("-> Saved top {} routes to {}", top_routes.len(), filename);

    } else {
        eprintln!("Could not find a nearby graph node to start from.");
    }

    Ok(())
}
