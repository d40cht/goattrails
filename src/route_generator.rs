use crate::{EdgeData, RouteGraph};
use geo::prelude::*;
use geo::{Haversine, Point as GeoPoint};
use indicatif::{ProgressBar, ProgressStyle};
use petgraph::algo::{astar, dijkstra};
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use rand::Rng;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone)]
struct CandidateSegment {
    start_node: NodeIndex,
    end_node: NodeIndex,
    distance: f64,
    ascent: f64,
}

pub fn generate_route(
    graph: &RouteGraph,
    start_node: NodeIndex,
    target_distance: f64,
    _iteration_limit: usize, // iteration_limit is not used in the new algo
) -> Option<Vec<EdgeData>> {
    const N_CANDIDATE_SEGMENTS: usize = 500;
    const MIN_ASCENT_METERS: f64 = 5.0;

    println!("Starting route generation with greedy insertion heuristic...");

    let start_point_coords = graph.node_weight(start_node).unwrap();
    let start_point_geo = GeoPoint::new(start_point_coords.lon, start_point_coords.lat);
    let search_radius = target_distance / 3.0;

    // Step 1: Candidate Segment Identification
    let mut candidate_segments: Vec<_> = graph
        .edge_references()
        .filter_map(|edge| {
            let weight = edge.weight();
            if weight.distance == 0.0 {
                return None;
            }
            let source_node_coords = graph.node_weight(edge.source()).unwrap();
            let distance_from_start =
                Haversine.distance(start_point_geo, GeoPoint::new(source_node_coords.lon, source_node_coords.lat));

            if distance_from_start > search_radius || weight.ascent < MIN_ASCENT_METERS {
                return None;
            }

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

    candidate_segments.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    let top_candidates: Vec<CandidateSegment> =
        candidate_segments.into_iter().map(|(seg, _)| seg).take(N_CANDIDATE_SEGMENTS).collect();

    if top_candidates.is_empty() {
        eprintln!("No suitable candidate segments found within the search radius.");
        return None;
    }
    println!("Identified {} candidate segments.", top_candidates.len());

    // Step 2: Pre-computation of Shortest Paths (APSP)
    println!("\nPre-computing shortest paths between key nodes...");
    let mut key_nodes: HashSet<NodeIndex> = top_candidates.iter().flat_map(|s| [s.start_node, s.end_node]).collect();
    key_nodes.insert(start_node);
    let key_nodes_vec: Vec<NodeIndex> = key_nodes.into_iter().collect();

    let mut distance_matrix = HashMap::new();
    let bar = ProgressBar::new(key_nodes_vec.len() as u64);
    bar.set_style(ProgressStyle::default_bar().template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta})").unwrap().progress_chars("#>-"));

    for &from_node in &key_nodes_vec {
        bar.inc(1);
        let shortest_paths = dijkstra(graph, from_node, None, |e| e.weight().distance);
        for &to_node in &key_nodes_vec {
            if let Some(distance) = shortest_paths.get(&to_node) {
                distance_matrix.insert((from_node, to_node), *distance);
            }
        }
    }
    bar.finish_with_message("APSP calculation complete.");
    println!("Distance matrix has {} entries.", distance_matrix.len());

    // Step 3: Route Calculation using Greedy Insertion
    println!("\nCalculating best route using greedy insertion...");
    let mut tour: Vec<CandidateSegment> = Vec::new();
    let mut current_properties = (0.0, 0.0);

    let mut remaining_candidates = top_candidates;
    let mut rng = rand::thread_rng();

    let bar = ProgressBar::new(remaining_candidates.len() as u64);
    bar.set_style(ProgressStyle::default_bar().template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta}) {msg}").unwrap().progress_chars("#>-"));

    while !remaining_candidates.is_empty() {
        bar.inc(1);

        const K_TOP_CANDIDATES_TO_CONSIDER: usize = 5;
        let pool_size = std::cmp::min(K_TOP_CANDIDATES_TO_CONSIDER, remaining_candidates.len());

        let random_candidate_pool_idx = rng.gen_range(0..pool_size);
        let candidate = remaining_candidates.remove(random_candidate_pool_idx);

        if tour.is_empty() {
            if let Some((dist, ascent)) = calculate_tour_properties_from_segments(start_node, &[candidate.clone()], &distance_matrix) {
                if dist <= target_distance {
                    tour.push(candidate);
                    current_properties = (dist, ascent);
                }
            }
            continue;
        }

        let mut best_insertion_idx = 0;
        let mut min_cost = f64::INFINITY;
        let mut best_new_props = (0.0, 0.0);

        for i in 0..=tour.len() {
            let mut temp_tour = tour.clone();
            temp_tour.insert(i, candidate.clone());
            if let Some((new_dist, new_ascent)) =
                calculate_tour_properties_from_segments(start_node, &temp_tour, &distance_matrix)
            {
                if new_dist <= target_distance {
                    let cost = new_dist - current_properties.0;
                    if cost < min_cost {
                        min_cost = cost;
                        best_insertion_idx = i;
                        best_new_props = (new_dist, new_ascent);
                    }
                }
            }
        }

        if min_cost != f64::INFINITY {
            tour.insert(best_insertion_idx, candidate);
            current_properties = best_new_props;
            bar.set_message(format!("Dist: {:.2}km, Ascent: {:.1}m", current_properties.0 / 1000.0, current_properties.1));
        }
    }
    bar.finish();

    if tour.is_empty() {
        eprintln!("Could not construct a route with the given constraints.");
        return None;
    }
    println!("Final tour has {} segments.", tour.len());
    println!("Final route estimate: Distance = {:.2}km, Ascent = {:.2}m", current_properties.0 / 1000.0, current_properties.1);

    // Step 4: Reconstruct final path
    println!("\nReconstructing final route path...");

    // Create a set of discouraged reverse edges
    let discouraged_edges: HashSet<(NodeIndex, NodeIndex)> = tour
        .iter()
        .map(|segment| (segment.end_node, segment.start_node))
        .collect();

    let mut final_path_nodes = vec![start_node];
    let mut current_node = start_node;
    let path_bar = ProgressBar::new(tour.len() as u64 + 1);
    path_bar.set_style(ProgressStyle::default_bar().template("{spinner:.green} Reconstructing path... [{bar:40.cyan/blue}] {pos}/{len}").unwrap().progress_chars("#>-"));

    for segment in &tour {
        path_bar.inc(1);
        if let Some((_, path)) = astar(
            graph,
            current_node,
            |n| n == segment.start_node,
            |e| {
                if discouraged_edges.contains(&(e.source(), e.target())) {
                    e.weight().distance * 4.0
                } else {
                    e.weight().distance
                }
            },
            |_| 0.0,
        ) {
            final_path_nodes.extend(&path[1..]);
        } else {
            eprintln!("Could not find path between {:?} and {:?}", current_node, segment.start_node);
            return None;
        }
        final_path_nodes.push(segment.end_node);
        current_node = segment.end_node;
    }

    path_bar.inc(1);
    if let Some((_, path)) = astar(
        graph,
        current_node,
        |n| n == start_node,
        |e| {
            if discouraged_edges.contains(&(e.source(), e.target())) {
                e.weight().distance * 4.0
            } else {
                e.weight().distance
            }
        },
        |_| 0.0,
    ) {
        final_path_nodes.extend(&path[1..]);
    } else {
        eprintln!("Could not find path back to start node from {:?}", current_node);
        return None;
    }
    path_bar.finish();

    println!("Final path has {} nodes.", final_path_nodes.len());
    Some(build_edge_data_path(graph, &final_path_nodes))
}

fn calculate_tour_properties_from_segments(
    start_node: NodeIndex,
    tour: &[CandidateSegment],
    distance_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
) -> Option<(f64, f64)> {
    if tour.is_empty() { return Some((0.0, 0.0)); }
    let mut total_distance = 0.0;
    let mut total_ascent = 0.0;
    let mut current_node = start_node;

    for segment in tour {
        total_distance += distance_matrix.get(&(current_node, segment.start_node))?;
        total_distance += segment.distance;
        total_ascent += segment.ascent;
        current_node = segment.end_node;
    }
    total_distance += distance_matrix.get(&(current_node, start_node))?;
    Some((total_distance, total_ascent))
}

fn build_edge_data_path(graph: &RouteGraph, nodes: &[NodeIndex]) -> Vec<EdgeData> {
    let mut path = Vec::new();
    for i in 0..nodes.len() - 1 {
        if let Some(edge_ref) = graph.find_edge(nodes[i], nodes[i+1]) {
            path.push(graph.edge_weight(edge_ref).unwrap().clone());
        } else {
            eprintln!("Error: No direct edge found between nodes {:?} and {:?}. This indicates a broken path.", nodes[i], nodes[i+1]);
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
        let n2 = graph.add_node(Point { lat: 0.0, lon: 0.01 });
        let n3 = graph.add_node(Point { lat: 0.01, lon: 0.01 });
        let n4 = graph.add_node(Point { lat: 0.01, lon: 0.0 });

        graph.add_edge(n1, n2, EdgeData { segment_id: 1, path: vec![], distance: 1000.0, ascent: 10.0, descent: 5.0, start_node: n1, end_node: n2 });
        graph.add_edge(n2, n1, EdgeData { segment_id: 1, path: vec![], distance: 1000.0, ascent: 5.0, descent: 10.0, start_node: n2, end_node: n1 });
        graph.add_edge(n2, n3, EdgeData { segment_id: 2, path: vec![], distance: 1000.0, ascent: 20.0, descent: 0.0, start_node: n2, end_node: n3 });
        graph.add_edge(n3, n2, EdgeData { segment_id: 2, path: vec![], distance: 1000.0, ascent: 0.0, descent: 20.0, start_node: n3, end_node: n2 });
        graph.add_edge(n3, n4, EdgeData { segment_id: 3, path: vec![], distance: 1000.0, ascent: 30.0, descent: 10.0, start_node: n3, end_node: n4 });
        graph.add_edge(n4, n3, EdgeData { segment_id: 3, path: vec![], distance: 1000.0, ascent: 10.0, descent: 30.0, start_node: n4, end_node: n3 });
        graph.add_edge(n4, n1, EdgeData { segment_id: 4, path: vec![], distance: 1000.0, ascent: 40.0, descent: 0.0, start_node: n4, end_node: n1 });
        graph.add_edge(n1, n4, EdgeData { segment_id: 4, path: vec![], distance: 1000.0, ascent: 0.0, descent: 40.0, start_node: n1, end_node: n4 });
        graph.add_edge(n1, n3, EdgeData { segment_id: 5, path: vec![], distance: 1414.0, ascent: 5.0, descent: 5.0, start_node: n1, end_node: n3 });
        graph.add_edge(n3, n1, EdgeData { segment_id: 5, path: vec![], distance: 1414.0, ascent: 5.0, descent: 5.0, start_node: n3, end_node: n1 });
        (graph, n1)
    }

    #[test]
    fn test_new_generate_route_returns_a_route() {
        let (graph, start_node) = create_test_graph();
        let route = generate_route(&graph, start_node, 10000.0, 100);
        assert!(route.is_some());
        let route_path = route.unwrap();
        assert!(!route_path.is_empty());
        let total_dist: f64 = route_path.iter().map(|e| e.distance).sum();
        assert!(total_dist > 0.0);
    }
}
