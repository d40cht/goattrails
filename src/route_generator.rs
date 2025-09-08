use crate::{EdgeData, Point, RouteGraph};
use petgraph::graph::{NodeIndex, EdgeIndex};
use petgraph::algo::astar;
use petgraph::visit::EdgeRef;
use rand::Rng;
use rand::seq::SliceRandom;
use geo::HaversineDistance;
use std::collections::HashSet;

struct Route {
    nodes: Vec<NodeIndex>,
    distance: f64,
    ascent: f64,
}

#[derive(Clone)]
struct CandidateExpansion {
    nodes: Vec<NodeIndex>,
    distance: f64,
    ascent: f64,
    score: f64,
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
    const N_CANDIDATES: usize = 10;
    const M_TOP_CANDIDATES: usize = 3;

    let initial_route_nodes = if let Some(neighbor) = find_nearest_neighbor(graph, start_node) {
        vec![start_node, neighbor, start_node]
    } else {
        return None;
    };

    let (initial_dist, initial_ascent) = calculate_route_properties(graph, &initial_route_nodes);
    let mut current_route = Route {
        nodes: initial_route_nodes,
        distance: initial_dist,
        ascent: initial_ascent,
    };

    let mut used_edges: HashSet<EdgeIndex> = HashSet::new();
    update_used_edges(graph, &current_route.nodes, &mut used_edges);

    println!("Starting route generation. Initial loop distance: {:.2}m", current_route.distance);

    for i in 0..iteration_limit {
        if current_route.distance >= target_distance {
            println!("\nTarget distance reached after {} iterations.", i);
            break;
        }

        let mut candidates: Vec<CandidateExpansion> = Vec::new();

        for _ in 0..N_CANDIDATES {
            if current_route.nodes.len() < 2 { continue; }

            let segment_start_idx = rng.gen_range(0..current_route.nodes.len() - 1);
            let segment_end_idx = segment_start_idx + 1;
            let u = current_route.nodes[segment_start_idx];
            let v = current_route.nodes[segment_end_idx];

            if u == v { continue; }

            let original_segment_dist = calculate_route_properties(graph, &current_route.nodes[segment_start_idx..=segment_end_idx]).0;

            if let Some((new_path_nodes, (new_dist, new_ascent))) = find_alternative_path(graph, u, v, original_segment_dist, &used_edges) {

                let mut potential_new_route_nodes = current_route.nodes.clone();
                potential_new_route_nodes.splice(segment_start_idx..=segment_end_idx, new_path_nodes);

                let (total_dist, total_ascent) = calculate_route_properties(graph, &potential_new_route_nodes);

                let score = total_ascent;

                candidates.push(CandidateExpansion {
                    nodes: potential_new_route_nodes,
                    distance: total_dist,
                    ascent: total_ascent,
                    score,
                });
            }
        }

        if candidates.is_empty() {
            continue;
        }

        candidates.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        let top_m = candidates.iter().take(M_TOP_CANDIDATES).collect::<Vec<_>>();

        if let Some(chosen_candidate) = top_m.choose(&mut rng) {
            current_route.nodes = chosen_candidate.nodes.clone();
            current_route.distance = chosen_candidate.distance;
            current_route.ascent = chosen_candidate.ascent;

            used_edges.clear();
            update_used_edges(graph, &current_route.nodes, &mut used_edges);

            print!(".");
            if i % 50 == 0 {
                println!("\nIter {}: Dist={:.2}km, Asc={:.1}m", i, current_route.distance / 1000.0, current_route.ascent);
            }
        }
    }

    println!("\nRoute generation finished. Final distance: {:.2}km, Ascent: {:.2}m", current_route.distance / 1000.0, current_route.ascent);

    Some(build_edge_data_path(graph, &current_route.nodes))
}

fn find_alternative_path(
    graph: &RouteGraph,
    start: NodeIndex,
    end: NodeIndex,
    original_distance: f64,
    used_edges: &HashSet<EdgeIndex>,
) -> Option<(Vec<NodeIndex>, (f64, f64))> {
    let max_distance = original_distance * 4.0 + 1000.0;
    let end_point = graph[end];

    let result = astar(
        graph,
        start,
        |finish| finish == end,
        |e| {
            if used_edges.contains(&e.id()) {
                return f64::INFINITY;
            }
            let weight = e.weight();
            weight.distance / (weight.ascent + 1.0)
        },
        |n| {
            let p1 = graph[n];
            let p1_geo = geo::Point::new(p1.lon, p1.lat);
            let p2_geo = geo::Point::new(end_point.lon, end_point.lat);
            p1_geo.haversine_distance(&p2_geo)
        },
    );

    if let Some((_, path_nodes)) = result {
        if path_nodes.len() > 2 {
            let (actual_dist, actual_ascent) = calculate_route_properties(graph, &path_nodes);
            if actual_dist < max_distance {
                return Some((path_nodes, (actual_dist, actual_ascent)));
            }
        }
    }

    None
}

fn update_used_edges(graph: &RouteGraph, nodes: &[NodeIndex], used_edges: &mut HashSet<EdgeIndex>) {
    for i in 0..nodes.len() - 1 {
        let u = nodes[i];
        let v = nodes[i+1];
        if let Some(edge_ref) = graph.edges(u).find(|e| e.target() == v) {
            used_edges.insert(edge_ref.id());
        }
    }
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

fn build_edge_data_path(graph: &RouteGraph, nodes: &[NodeIndex]) -> Vec<EdgeData> {
    let mut path = Vec::new();
    for i in 0..nodes.len() - 1 {
        let u = nodes[i];
        let v = nodes[i+1];
        if let Some(edge_ref) = graph.edges(u).find(|e| e.target() == v) {
            path.push(edge_ref.weight().clone());
        } else {
            eprintln!("Error: No direct edge found between nodes {:?} and {:?}. This indicates a broken path.", u, v);
            // Return empty path instead of panicking
            return Vec::new();
        }
    }
    path
}
