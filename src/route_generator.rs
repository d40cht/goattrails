use crate::{EdgeData, RouteGraph};
use petgraph::graph::{NodeIndex};
use petgraph::algo::astar;
use petgraph::visit::EdgeRef;
use rand::Rng;
use rand::seq::SliceRandom;
use geo::prelude::*;
use geo::Haversine;
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

            let forbidden_nodes: HashSet<NodeIndex> = current_route.nodes.iter().cloned().collect();

            if let Some((new_path_nodes, (_new_dist, _new_ascent))) = find_alternative_path(graph, u, v, original_segment_dist, &forbidden_nodes) {

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
    forbidden_nodes: &HashSet<NodeIndex>,
) -> Option<(Vec<NodeIndex>, (f64, f64))> {
    let max_distance = original_distance * 4.0 + 1000.0;
    let end_point = graph[end];

    let result = astar(
        graph,
        start,
        |finish| finish == end,
        |e| {
            // Prevent taking the direct edge back
            if e.source() == start && e.target() == end {
                return f64::INFINITY;
            }

            let target_node = e.target();
            if forbidden_nodes.contains(&target_node) && target_node != end {
                return f64::INFINITY;
            }

            let cost = e.weight().distance * e.weight().penalty;
            cost / (e.weight().ascent + 1.0)
        },
        |n| {
            let p1 = graph[n];
            let p1_geo = geo::Point::new(p1.lon, p1.lat);
            let p2_geo = geo::Point::new(end_point.lon, end_point.lat);
            Haversine.distance(p1_geo, p2_geo)
        },
    );

    if let Some((_total_dist, path_nodes)) = result {
        let (actual_dist, actual_ascent) = calculate_route_properties(graph, &path_nodes);
        if actual_dist < max_distance {
            return Some((path_nodes, (actual_dist, actual_ascent)));
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Point, EdgeData, RouteGraph};
    use petgraph::graph::NodeIndex;

    fn create_test_graph() -> (RouteGraph, NodeIndex) {
        let mut graph = RouteGraph::new();
        let n1 = graph.add_node(Point { lat: 0.0, lon: 0.0 });
        let n2 = graph.add_node(Point { lat: 0.0, lon: 0.1 });
        let n3 = graph.add_node(Point { lat: 0.1, lon: 0.1 });
        let n4 = graph.add_node(Point { lat: 0.1, lon: 0.0 });

        graph.add_edge(n1, n2, EdgeData { segment_id: 1, path: vec![], distance: 1000.0, ascent: 10.0, descent: 5.0, penalty: 1.0 });
        graph.add_edge(n2, n1, EdgeData { segment_id: 1, path: vec![], distance: 1000.0, ascent: 5.0, descent: 10.0, penalty: 1.0 });

        graph.add_edge(n2, n3, EdgeData { segment_id: 2, path: vec![], distance: 1000.0, ascent: 20.0, descent: 0.0, penalty: 1.0 });
        graph.add_edge(n3, n2, EdgeData { segment_id: 2, path: vec![], distance: 1000.0, ascent: 0.0, descent: 20.0, penalty: 1.0 });

        graph.add_edge(n3, n4, EdgeData { segment_id: 3, path: vec![], distance: 1000.0, ascent: 30.0, descent: 10.0, penalty: 1.0 });
        graph.add_edge(n4, n3, EdgeData { segment_id: 3, path: vec![], distance: 1000.0, ascent: 10.0, descent: 30.0, penalty: 1.0 });

        graph.add_edge(n4, n1, EdgeData { segment_id: 4, path: vec![], distance: 1000.0, ascent: 40.0, descent: 0.0, penalty: 1.0 });
        graph.add_edge(n1, n4, EdgeData { segment_id: 4, path: vec![], distance: 1000.0, ascent: 0.0, descent: 40.0, penalty: 1.0 });

        graph.add_edge(n1, n3, EdgeData { segment_id: 5, path: vec![], distance: 1414.0, ascent: 5.0, descent: 5.0, penalty: 1.0 });
        graph.add_edge(n3, n1, EdgeData { segment_id: 5, path: vec![], distance: 1414.0, ascent: 5.0, descent: 5.0, penalty: 1.0 });

        (graph, n1)
    }

    #[test]
    fn test_generate_route_returns_a_route() {
        let (graph, start_node) = create_test_graph();
        let route = generate_route(&graph, start_node, 5000.0, 100);
        assert!(route.is_some());
        let route_path = route.unwrap();
        assert!(!route_path.is_empty());
    }

    #[test]
    fn test_find_alternative_path() {
        let (graph, n1) = create_test_graph();
        let n2 = NodeIndex::new(1);
        let forbidden_nodes = HashSet::new();
        let result = find_alternative_path(&graph, n1, n2, 1000.0, &forbidden_nodes);
        assert!(result.is_some());
        let (path, (dist, ascent)) = result.unwrap();
        assert_eq!(path, vec![NodeIndex::new(0), NodeIndex::new(2), NodeIndex::new(1)]);
        assert!((dist - 2414.0).abs() < 1e-9);
        assert!((ascent - 5.0).abs() < 1e-9);
    }
}
