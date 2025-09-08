use crate::{EdgeData, RouteGraph};
use petgraph::graph::{NodeIndex};
use petgraph::algo::astar;
use petgraph::visit::EdgeRef;
use rand::Rng;
use rand::seq::SliceRandom;
use geo::HaversineDistance;
use std::collections::HashSet;
use indicatif::{ProgressBar, ProgressStyle};

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

    let bar = ProgressBar::new(iteration_limit as u64);
    bar.set_style(ProgressStyle::default_bar()
        .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta}) {msg}")
        .unwrap()
        .progress_chars("#>-"));

    println!("Starting route generation...");

    for _ in 0..iteration_limit {
        if current_route.distance >= target_distance {
            bar.finish_with_message("Target distance reached.");
            break;
        }

        bar.set_message(format!("Dist: {:.2}km, Ascent: {:.1}m", current_route.distance / 1000.0, current_route.ascent));
        bar.inc(1);

        let mut candidates: Vec<CandidateExpansion> = Vec::new();

        for _ in 0..N_CANDIDATES {
            if current_route.nodes.len() < 2 { continue; }

            let segment_start_idx = rng.gen_range(0..current_route.nodes.len() - 1);
            let segment_end_idx = segment_start_idx + 1;
            let u = current_route.nodes[segment_start_idx];
            let v = current_route.nodes[segment_end_idx];

            if u == v { continue; }

            let original_segment_dist = calculate_route_properties(graph, &current_route.nodes[segment_start_idx..=segment_end_idx]).0;

            let mut forbidden_segment_ids = HashSet::new();
            if let Some(edge_to_replace) = graph.find_edge(u, v) {
                forbidden_segment_ids.insert(graph[edge_to_replace].segment_id);
            }

            if let Some((new_path_nodes, (new_dist, new_ascent))) = find_alternative_path(graph, u, v, original_segment_dist, &forbidden_segment_ids) {

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
        }
    }
    bar.finish();
    println!("Route generation finished. Final distance: {:.2}km, Ascent: {:.2}m", current_route.distance / 1000.0, current_route.ascent);

    Some(build_edge_data_path(graph, &current_route.nodes))
}

fn find_alternative_path(
    graph: &RouteGraph,
    start: NodeIndex,
    end: NodeIndex,
    original_distance: f64,
    forbidden_segment_ids: &HashSet<u32>,
) -> Option<(Vec<NodeIndex>, (f64, f64))> {
    let max_distance = original_distance * 4.0 + 1000.0;
    let end_point = graph[end];

    let result = astar(
        graph,
        start,
        |finish| finish == end,
        |e| {
            if forbidden_segment_ids.contains(&e.weight().segment_id) {
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
