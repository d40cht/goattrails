use crate::{EdgeData, Point, RouteGraph};
use petgraph::graph::NodeIndex;
use petgraph::algo::astar;
use petgraph::visit::EdgeRef;
use rand::Rng;
use geo::HaversineDistance;

// Represents the current route state
struct Route {
    nodes: Vec<NodeIndex>,
    distance: f64,
    ascent: f64,
}

fn find_nearest_neighbor(graph: &RouteGraph, start_node: NodeIndex) -> Option<NodeIndex> {
    graph.edges(start_node)
        .min_by(|a, b| a.weight().distance.partial_cmp(&b.weight().distance).unwrap())
        .map(|edge| edge.target())
}

pub fn generate_route(
    graph: &RouteGraph,
    start_node: NodeIndex,
    target_distance: f64,
    iteration_limit: usize,
) -> Option<Vec<EdgeData>> {
    let mut rng = rand::thread_rng();

    // --- Create the initial seed loop ---
    let initial_route_nodes = if let Some(neighbor) = find_nearest_neighbor(graph, start_node) {
        vec![start_node, neighbor, start_node]
    } else {
        println!("Could not find a neighbor for the start node. Cannot generate a route.");
        return None;
    };

    let (initial_dist, initial_ascent) = calculate_route_properties(graph, &initial_route_nodes);
    let mut current_route = Route {
        nodes: initial_route_nodes,
        distance: initial_dist,
        ascent: initial_ascent,
    };

    println!("Starting route generation from node {:?}. Initial loop distance: {:.2}m", start_node, current_route.distance);

    for i in 0..iteration_limit {
        if current_route.distance >= target_distance {
            println!("\nTarget distance of {:.2}km reached after {} iterations.", target_distance / 1000.0, i);
            break;
        }

        if current_route.nodes.len() < 2 {
            println!("Route has less than 2 nodes, stopping.");
            break;
        }

        // 1. Select a random segment to replace
        let segment_start_idx = rng.gen_range(0..current_route.nodes.len() - 1);
        let segment_end_idx = segment_start_idx + 1;
        let u = current_route.nodes[segment_start_idx];
        let v = current_route.nodes[segment_end_idx];

        if u == v { continue; }

        let (original_dist, original_ascent) = calculate_route_properties(graph, &current_route.nodes[segment_start_idx..=segment_end_idx]);

        // 2. Find an alternative path
        if let Some((new_path_nodes, (new_dist, new_ascent))) = find_alternative_path(graph, u, v, original_dist) {
            let original_ratio = if original_dist > 0.0 { original_ascent / original_dist } else { 0.0 };
            let new_ratio = if new_dist > 0.0 { new_ascent / new_dist } else { 0.0 };

            if new_ratio > original_ratio {
                current_route.nodes.splice(segment_start_idx..=segment_end_idx, new_path_nodes);

                let (total_dist, total_ascent) = calculate_route_properties(graph, &current_route.nodes);
                current_route.distance = total_dist;
                current_route.ascent = total_ascent;

                print!(".");
                if i % 100 == 0 {
                    println!("\nIter {}: Dist={:.2}km, Asc={:.1}m", i, total_dist / 1000.0, total_ascent);
                }
            }
        }
    }

    println!("\nRoute generation finished. Final distance: {:.2}km, Ascent: {:.2}m", current_route.distance / 1000.0, current_route.ascent);

    if current_route.nodes.len() <= 2 && current_route.distance == 0.0 {
        return None;
    }
    build_edge_data_path(graph, &current_route.nodes)
}

fn find_alternative_path(
    graph: &RouteGraph,
    start: NodeIndex,
    end: NodeIndex,
    original_distance: f64,
) -> Option<(Vec<NodeIndex>, (f64, f64))> {
    let max_distance = original_distance * 2.0;
    let end_point = graph[end];

    let result = astar(
        graph,
        start,
        |finish| finish == end,
        |e| {
            let weight = e.weight();
            // A cost function that prefers high ascent for a given distance.
            // Add a small epsilon to avoid division by zero for flat segments.
            weight.distance / (weight.ascent + 1.0)
        },
        |n| {
            let p1 = graph[n];
            let p1_geo = geo::Point::new(p1.lon, p1.lat);
            let p2_geo = geo::Point::new(end_point.lon, end_point.lat);
            p1_geo.haversine_distance(&p2_geo)
        },
    );

    if let Some((path_dist, path_nodes)) = result {
        if path_nodes.len() > 2 && path_dist < max_distance {
            let props = calculate_route_properties(graph, &path_nodes);
            return Some((path_nodes, props));
        }
    }

    None
}

fn calculate_route_properties(graph: &RouteGraph, nodes: &[NodeIndex]) -> (f64, f64) {
    let mut distance = 0.0;
    let mut ascent = 0.0;
    for i in 0..nodes.len() - 1 {
        let u = nodes[i];
        let v = nodes[i+1];
        if let Some(edge_ref) = graph.edges(u).find(|e| e.target() == v) {
            let edge_data = edge_ref.weight();
            distance += edge_data.distance;
            ascent += edge_data.ascent;
        }
    }
    (distance, ascent)
}

fn build_edge_data_path(graph: &RouteGraph, nodes: &[NodeIndex]) -> Option<Vec<EdgeData>> {
    let mut path = Vec::new();
    for i in 0..nodes.len() - 1 {
        let u = nodes[i];
        let v = nodes[i+1];
        if let Some(edge_ref) = graph.edges(u).find(|e| e.target() == v) {
            path.push(edge_ref.weight().clone());
        } else {
            eprintln!("Error: No direct edge found between nodes {:?} and {:?}. This indicates a broken path.", u, v);
            return None;
        }
    }
    Some(path)
}
