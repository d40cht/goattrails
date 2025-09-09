use std::collections::{HashMap, HashSet};
use geo::prelude::*;
use geo::{Point as GeoPoint, Haversine};
use indicatif::{ProgressBar, ProgressStyle};
use ndarray::Array2;
use osmpbf::{Element, ElementReader};
use petgraph::algo::dijkstra;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::visit::EdgeRef;
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::Read;
use tiff::decoder::{Decoder, DecodingResult};
use tiff::tags::Tag;
use clap::Parser;
use serde::Deserialize;

pub mod map_exporter;
pub mod route_generator;

/// Hilly route finder
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Path to the configuration file
    #[arg(short, long, default_value_t = String::from("configs/dev.toml"))]
    config: String,

    /// Enable verbose output
    #[arg(short, long, default_value_t = false)]
    verbose: bool,
}

#[derive(Deserialize)]
struct GlobalConfig {
    osm_path: String,
    srtm_path: String,
    map_offset_scale: Option<f64>,
}

#[derive(Deserialize, Clone, Copy)]
pub struct AlgorithmConfig {
    pub n_candidate_segments: usize,
    pub search_radius_divisor: f64,
    pub k_top_candidates_to_consider: usize,
    pub m_candidates_to_evaluate: usize,
    pub penalty_factor: f64,
}

#[derive(Deserialize)]
struct RouteConfig {
    lat: f64,
    lon: f64,
    target_distance_km: f64,
    num_candidate_routes: usize,
}

#[derive(Deserialize)]
struct Config {
    global: GlobalConfig,
    algorithm: AlgorithmConfig,
    routes: HashMap<String, RouteConfig>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub lat: f64,
    pub lon: f64,
}

#[derive(Debug, Clone)]
pub struct EdgeData {
    pub segment_id: u32,
    pub path: Vec<Point>,
    pub distance: f64,
    pub weighted_distance: f64,
    pub ascent: f64,
    pub descent: f64,
    pub start_node: NodeIndex,
    pub end_node: NodeIndex,
}

type RouteGraph = Graph<Point, EdgeData, petgraph::Directed>;

#[derive(Debug, Clone)]
pub struct CandidateSegment {
    pub start_node: NodeIndex,
    pub end_node: NodeIndex,
    pub distance: f64,
    pub ascent: f64,
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

    let mut segment_id_counter = 0u32;
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

                            let forward_edge = EdgeData {
                                segment_id: segment_id_counter,
                                path: current_path_segment.clone(),
                                distance: 0.0,
                                weighted_distance: 0.0,
                                ascent: 0.0,
                                descent: 0.0,
                                start_node: start_idx,
                                end_node: end_idx,
                            };

                            let mut reversed_path = current_path_segment.clone();
                            reversed_path.reverse();
                            let reverse_edge = EdgeData {
                                segment_id: segment_id_counter,
                                path: reversed_path,
                                distance: 0.0,
                                weighted_distance: 0.0,
                                ascent: 0.0,
                                descent: 0.0,
                                start_node: end_idx,
                                end_node: start_idx,
                            };

                            graph.add_edge(start_idx, end_idx, forward_edge);
                            graph.add_edge(end_idx, start_idx, reverse_edge);

                            segment_id_counter += 1;
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
            let segment_distance = Haversine.distance(p1, p2);
            distance += segment_distance;

            let num_steps = (segment_distance / INTERPOLATION_DISTANCE_M).ceil() as usize;
            if num_steps == 0 { continue; }

            for i in 0..=num_steps {
                let fraction = if num_steps > 0 { i as f64 / num_steps as f64 } else { 0.0 };
                let intermediate_geo_point = Haversine.point_at_ratio_between(p1, p2, fraction);
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
        edge.weighted_distance = distance;
        edge.ascent = ascent;
        edge.descent = descent;
    }

    // The reverse edges were created with ascent/descent as 0.0, now we need to fix them.
    let mut edges_to_update = Vec::new();
    for edge_ref in graph.edge_references() {
        if let Some(reverse_edge_index) = graph.find_edge(edge_ref.target(), edge_ref.source()) {
            if graph[reverse_edge_index].ascent == 0.0 && graph[reverse_edge_index].descent == 0.0 {
                let forward_weight = edge_ref.weight();
                edges_to_update.push((reverse_edge_index, forward_weight.descent, forward_weight.ascent));
            }
        }
    }
    for (edge_index, ascent, descent) in edges_to_update {
        if let Some(edge_weight) = graph.edge_weight_mut(edge_index) {
            edge_weight.ascent = ascent;
            edge_weight.descent = descent;
        }
    }

    println!("Graph build complete. Final graph has {} nodes and {} edges.", graph.node_count(), graph.edge_count());
    Ok((graph, parking_locations))
}

fn find_nearest_node(graph: &RouteGraph, point: &Point) -> Option<NodeIndex> {
    let point_geo = GeoPoint::new(point.lon, point.lat);
    graph.node_indices().min_by(|&a, &b| {
        let p_a = graph[a];
        let p_b = graph[b];
        let dist_a = Haversine.distance(GeoPoint::new(p_a.lon, p_a.lat), point_geo);
        let dist_b = Haversine.distance(GeoPoint::new(p_b.lon, p_b.lat), point_geo);
        dist_a.partial_cmp(&dist_b).unwrap()
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    println!("Loading config from: {}", args.config);
    let config_str = fs::read_to_string(args.config)?;
    let config: Config = toml::from_str(&config_str)?;

    let (mut graph, _parking_locations) = build_graph(&config.global.osm_path, &config.global.srtm_path)?;

    for (route_name, route_config) in config.routes {
        println!("\n--- Processing route: {} ---", route_name);

        let start_point = Point { lat: route_config.lat, lon: route_config.lon };
        println!("Using configured start point: ({:.4}, {:.4})", start_point.lat, start_point.lon);

        if let Some(start_node) = find_nearest_node(&graph, &start_point) {
            println!("Found nearest graph node. Identifying candidate segments...");
            const MIN_ASCENT_METERS: f64 = 5.0;

            let start_point_coords = graph.node_weight(start_node).unwrap();
            let start_point_geo = GeoPoint::new(start_point_coords.lon, start_point_coords.lat);
            let search_radius = (route_config.target_distance_km * 1000.0) / config.algorithm.search_radius_divisor;

            let mut candidate_segments_with_value: Vec<_> = graph
                .edge_references()
                .filter_map(|edge| {
                    let weight = edge.weight();
                    if weight.distance == 0.0 { return None; }
                    let source_node_coords = graph.node_weight(edge.source()).unwrap();
                    let distance_from_start = Haversine.distance(start_point_geo, GeoPoint::new(source_node_coords.lon, source_node_coords.lat));

                    if distance_from_start > search_radius || weight.ascent < MIN_ASCENT_METERS { return None; }

                    Some((
                        CandidateSegment {
                            start_node: edge.source(),
                            end_node: edge.target(),
                            distance: weight.distance,
                            ascent: weight.ascent,
                        },
                        weight.ascent / weight.distance,
                    ))
                })
                .collect();

            candidate_segments_with_value.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
            let top_candidates: Vec<CandidateSegment> = candidate_segments_with_value.into_iter().map(|(seg, _)| seg).take(config.algorithm.n_candidate_segments).collect();

            if top_candidates.is_empty() {
                eprintln!("No suitable candidate segments found within the search radius for route {}.", route_name);
                continue;
            }
            println!("Identified {} candidate segments. Applying penalties for APSP calculation...", top_candidates.len());

            for candidate in &top_candidates {
                if let Some(edge_index) = graph.find_edge(candidate.start_node, candidate.end_node) {
                    graph[edge_index].weighted_distance = graph[edge_index].distance * config.algorithm.penalty_factor;
                }
                if let Some(edge_index) = graph.find_edge(candidate.end_node, candidate.start_node) {
                    graph[edge_index].weighted_distance = graph[edge_index].distance * config.algorithm.penalty_factor;
                }
            }

            let (actual_distance_matrix, cost_matrix) = {
                let mut key_nodes: HashSet<NodeIndex> = top_candidates.iter().flat_map(|s| [s.start_node, s.end_node]).collect();
                key_nodes.insert(start_node);
                let key_nodes_vec: Vec<NodeIndex> = key_nodes.into_iter().collect();
                let immutable_graph: &RouteGraph = &graph;

                println!("\nPre-computing actual shortest paths between key nodes...");
                let mut actual_distance_matrix = HashMap::new();
                let bar1 = ProgressBar::new(key_nodes_vec.len() as u64);
                bar1.set_style(ProgressStyle::default_bar().template("{spinner:.green} [1/2] [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta})").unwrap().progress_chars("#>-"));
                for &from_node in &key_nodes_vec {
                    bar1.inc(1);
                    let shortest_paths = dijkstra(immutable_graph, from_node, None, |e| e.weight().distance);
                    for &to_node in &key_nodes_vec {
                        if let Some(distance) = shortest_paths.get(&to_node) {
                            actual_distance_matrix.insert((from_node, to_node), *distance);
                        }
                    }
                }
                bar1.finish_with_message("Actual distance matrix complete.");

                println!("\nPre-computing cost-based shortest paths between key nodes...");
                let mut cost_matrix = HashMap::new();
                let bar2 = ProgressBar::new(key_nodes_vec.len() as u64);
                bar2.set_style(ProgressStyle::default_bar().template("{spinner:.green} [2/2] [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta})").unwrap().progress_chars("#>-"));
                for &from_node in &key_nodes_vec {
                    bar2.inc(1);
                    let shortest_paths = dijkstra(immutable_graph, from_node, None, |e| e.weight().weighted_distance);
                    for &to_node in &key_nodes_vec {
                        if let Some(distance) = shortest_paths.get(&to_node) {
                            cost_matrix.insert((from_node, to_node), *distance);
                        }
                    }
                }
                bar2.finish_with_message("Cost matrix complete.");
                (actual_distance_matrix, cost_matrix)
            };

            for candidate in &top_candidates {
                if let Some(edge_index) = graph.find_edge(candidate.start_node, candidate.end_node) {
                    graph[edge_index].weighted_distance = graph[edge_index].distance;
                }
                if let Some(edge_index) = graph.find_edge(candidate.end_node, candidate.start_node) {
                    graph[edge_index].weighted_distance = graph[edge_index].distance;
                }
            }

            let mut generated_routes = Vec::new();
            for i in 0..route_config.num_candidate_routes {
                println!("\n--- Generating Route Candidate {}/{} for {} ---", i + 1, route_config.num_candidate_routes, route_name);
                if let Some(route) = route_generator::generate_route(
                    &graph,
                    start_node,
                    route_config.target_distance_km * 1000.0,
                    &top_candidates,
                    &actual_distance_matrix,
                    &cost_matrix,
                    &config.algorithm,
                ) {
                    generated_routes.push(route);
                } else {
                    eprintln!("Failed to generate a route candidate.");
                }
            }

            if generated_routes.is_empty() {
                eprintln!("No routes were generated for {}.", route_name);
                continue;
            }

            println!("\n--- All routes generated for {}. Ranking by ascent... ---", route_name);
            generated_routes.sort_by(|a, b| {
                let a_ascent: f64 = a.iter().map(|e| e.ascent).sum();
                let b_ascent: f64 = b.iter().map(|e| e.ascent).sum();
                b_ascent.partial_cmp(&a_ascent).unwrap_or(std::cmp::Ordering::Equal)
            });

            let top_routes = generated_routes;

            println!("\n--- Top {} Routes for {} ---", top_routes.len(), route_name);
            for (i, route) in top_routes.iter().enumerate() {
                let total_dist: f64 = route.iter().map(|e| e.distance).sum();
                let total_ascent: f64 = route.iter().map(|e| e.ascent).sum();
                println!("\nRoute #{}: Distance = {:.2}km, Ascent = {:.2}m", i + 1, total_dist / 1000.0, total_ascent);

                if args.verbose {
                    println!("{:<15} | {:<12} | {:<10} | {:<10}", "Segment ID", "Distance (m)", "Ascent (m)", "Descent (m)");
                    println!("{:-<16}|{:-<14}|{:-<12}|{:-<12}", "", "", "", "");
                    for segment in route {
                        println!("{:<15} | {:<12.2} | {:<10.2} | {:<10.2}", segment.segment_id, segment.distance, segment.ascent, segment.descent);
                    }
                }
            }

            fs::create_dir_all("vis")?;
            let map_title = format!("Generated Routes for {}", route_name);
            let offset_scale = config.global.map_offset_scale.unwrap_or(0.000060);
            let html_content = map_exporter::export_route_map(&top_routes, &map_title, offset_scale);
            let filename = format!("vis/{}.html", route_name);
            fs::write(&filename, html_content)?;
            println!("-> Saved top {} routes to {}", top_routes.len(), filename);

        } else {
            eprintln!("Could not find a nearby graph node to start from for route {}.", route_name);
        }
    }

    Ok(())
}
