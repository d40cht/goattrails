use crate::{AlgorithmConfig, CandidateSegment, EdgeData, RouteGraph};
use indicatif::{ProgressBar, ProgressStyle};
use petgraph::algo::astar;
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use rand;
use std::collections::{HashMap, HashSet};

pub fn generate_route(
    graph: &RouteGraph,
    start_node: NodeIndex,
    target_distance: f64,
    top_candidates: &[CandidateSegment],
    actual_distance_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
    cost_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
    config: &AlgorithmConfig,
) -> Option<Vec<EdgeData>> {
    println!("Starting route generation with greedy insertion heuristic...");

    let mut tour: Vec<CandidateSegment> = Vec::new();
    let mut current_properties = (0.0, 0.0);

    let mut remaining_candidates = top_candidates.to_vec();
    let mut rng = rand::thread_rng();

    let bar = ProgressBar::new(remaining_candidates.len() as u64);
    bar.set_style(ProgressStyle::default_bar().template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta}) {msg}").unwrap().progress_chars("#>-"));

    while !remaining_candidates.is_empty() {
        bar.inc(1);

        let k_pool_size = std::cmp::min(config.k_top_candidates_to_consider, remaining_candidates.len());
        if k_pool_size == 0 { break; }
        let m_to_evaluate = std::cmp::min(config.m_candidates_to_evaluate, k_pool_size);

        let candidate_indices_to_check = rand::seq::index::sample(&mut rng, k_pool_size, m_to_evaluate).into_vec();

        let mut best_insertion_info: Option<(usize, usize, f64, (f64, f64))> = None; // (candidate_index_in_remaining, insertion_index_in_tour, cost, new_props)

        for &candidate_idx in &candidate_indices_to_check {
            let candidate = &remaining_candidates[candidate_idx];

            if tour.is_empty() {
                if let Some((dist, ascent)) = calculate_tour_properties_from_segments(start_node, &[candidate.clone()], actual_distance_matrix) {
                    if dist <= target_distance {
                        if let Some((cost, _)) = calculate_tour_properties_from_segments(start_node, &[candidate.clone()], cost_matrix) {
                            if cost < best_insertion_info.map_or(f64::INFINITY, |(_, _, c, _)| c) {
                                best_insertion_info = Some((candidate_idx, 0, cost, (dist, ascent)));
                            }
                        }
                    }
                }
            } else {
                let mut best_insertion_for_this_candidate: Option<(usize, f64, (f64, f64))> = None;

                for i in 0..=tour.len() {
                    let mut temp_tour = tour.clone();
                    temp_tour.insert(i, candidate.clone());

                    if let Some((new_dist, new_ascent)) = calculate_tour_properties_from_segments(start_node, &temp_tour, actual_distance_matrix) {
                        if new_dist <= target_distance {
                            if let Some((new_cost, _)) = calculate_tour_properties_from_segments(start_node, &temp_tour, cost_matrix) {
                                let (current_cost, _) = calculate_tour_properties_from_segments(start_node, &tour, cost_matrix).unwrap_or((0.0, 0.0));
                                let cost = new_cost - current_cost;
                                if cost < best_insertion_for_this_candidate.map_or(f64::INFINITY, |(_, c, _)| c) {
                                    best_insertion_for_this_candidate = Some((i, cost, (new_dist, new_ascent)));
                                }
                            }
                        }
                    }
                }

                if let Some((insertion_idx, cost, new_props)) = best_insertion_for_this_candidate {
                    if cost < best_insertion_info.map_or(f64::INFINITY, |(_, _, c, _)| c) {
                        best_insertion_info = Some((candidate_idx, insertion_idx, cost, new_props));
                    }
                }
            }
        }

        if let Some((candidate_idx_to_insert, insertion_idx, _, new_props)) = best_insertion_info {
            let candidate = remaining_candidates.remove(candidate_idx_to_insert);
            tour.insert(insertion_idx, candidate);
            current_properties = new_props;
            bar.set_message(format!("Dist: {:.2}km, Ascent: {:.1}m", current_properties.0 / 1000.0, current_properties.1));
        } else {
            println!("No insertable candidates found in this iteration. Finalizing tour.");
            break;
        }
    }
    bar.finish();

    if tour.is_empty() {
        eprintln!("Could not construct a route with the given constraints.");
        return None;
    }
    println!("Final tour has {} segments.", tour.len());
    println!("Final route estimate: Distance = {:.2}km, Ascent = {:.2}m", current_properties.0 / 1000.0, current_properties.1);

    println!("\nReconstructing final route path...");
    let final_path_nodes = {
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
                    let mut cost = e.weight().weighted_distance;
                    if discouraged_edges.contains(&(e.source(), e.target())) {
                        cost *= config.penalty_factor;
                    }
                    cost
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
                let mut cost = e.weight().weighted_distance;
                if discouraged_edges.contains(&(e.source(), e.target())) {
                    cost *= config.penalty_factor;
                }
                cost
            },
            |_| 0.0,
        ) {
            final_path_nodes.extend(&path[1..]);
        } else {
            eprintln!("Could not find path back to start node from {:?}", current_node);
            return None;
        }
        path_bar.finish();
        final_path_nodes
    };

    println!("Final path has {} nodes.", final_path_nodes.len());
    Some(build_edge_data_path(graph, &final_path_nodes))
}

fn calculate_tour_properties_from_segments(
    start_node: NodeIndex,
    tour: &[CandidateSegment],
    matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
) -> Option<(f64, f64)> {
    if tour.is_empty() { return Some((0.0, 0.0)); }
    let mut total_distance = 0.0;
    let mut total_ascent = 0.0;
    let mut current_node = start_node;

    for segment in tour {
        total_distance += matrix.get(&(current_node, segment.start_node))?;
        total_distance += segment.distance;
        total_ascent += segment.ascent;
        current_node = segment.end_node;
    }
    total_distance += matrix.get(&(current_node, start_node))?;
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
    use crate::{Point, EdgeData, RouteGraph, AlgorithmConfig, CandidateSegment, HighwayType};
    use petgraph::graph::NodeIndex;
    use petgraph::algo::dijkstra;
    use std::collections::{HashMap, HashSet};

    fn create_test_graph() -> (RouteGraph, NodeIndex) {
        let mut graph = RouteGraph::new();
        let n1 = graph.add_node(Point { lat: 0.0, lon: 0.0 });
        let n2 = graph.add_node(Point { lat: 0.0, lon: 0.01 });
        let n3 = graph.add_node(Point { lat: 0.01, lon: 0.01 });
        let n4 = graph.add_node(Point { lat: 0.01, lon: 0.0 });

        graph.add_edge(n1, n2, EdgeData { segment_id: 1, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 10.0, descent: 5.0, start_node: n1, end_node: n2 });
        graph.add_edge(n2, n1, EdgeData { segment_id: 1, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 5.0, descent: 10.0, start_node: n2, end_node: n1 });
        graph.add_edge(n2, n3, EdgeData { segment_id: 2, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 20.0, descent: 0.0, start_node: n2, end_node: n3 });
        graph.add_edge(n3, n2, EdgeData { segment_id: 2, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 0.0, descent: 20.0, start_node: n3, end_node: n2 });
        graph.add_edge(n3, n4, EdgeData { segment_id: 3, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 30.0, descent: 10.0, start_node: n3, end_node: n4 });
        graph.add_edge(n4, n3, EdgeData { segment_id: 3, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 10.0, descent: 30.0, start_node: n4, end_node: n3 });
        graph.add_edge(n4, n1, EdgeData { segment_id: 4, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 40.0, descent: 0.0, start_node: n4, end_node: n1 });
        graph.add_edge(n1, n4, EdgeData { segment_id: 4, highway_type: HighwayType::Trail, path: vec![], distance: 1000.0, weighted_distance: 1000.0, ascent: 0.0, descent: 40.0, start_node: n1, end_node: n4 });
        graph.add_edge(n1, n3, EdgeData { segment_id: 5, highway_type: HighwayType::Trail, path: vec![], distance: 1414.0, weighted_distance: 1414.0, ascent: 5.0, descent: 5.0, start_node: n1, end_node: n3 });
        graph.add_edge(n3, n1, EdgeData { segment_id: 5, highway_type: HighwayType::Trail, path: vec![], distance: 1414.0, weighted_distance: 1414.0, ascent: 5.0, descent: 5.0, start_node: n3, end_node: n1 });
        (graph, n1)
    }

    #[test]
    fn test_new_generate_route_returns_a_route() {
        let (graph, start_node) = create_test_graph();
        let n2 = graph.node_indices().nth(1).unwrap();
        let n3 = graph.node_indices().nth(2).unwrap();
        let n4 = graph.node_indices().nth(3).unwrap();

        let top_candidates = vec![
             CandidateSegment { start_node: n2, end_node: n3, distance: 1000.0, ascent: 20.0 },
             CandidateSegment { start_node: n3, end_node: n4, distance: 1000.0, ascent: 30.0 },
        ];

        let mut key_nodes: HashSet<NodeIndex> = top_candidates.iter().flat_map(|s| [s.start_node, s.end_node]).collect();
        key_nodes.insert(start_node);
        let key_nodes_vec: Vec<NodeIndex> = key_nodes.into_iter().collect();

        let mut actual_distance_matrix = HashMap::new();
        for &from_node in &key_nodes_vec {
            let shortest_paths = dijkstra(&graph, from_node, None, |e| e.weight().distance);
            for &to_node in &key_nodes_vec {
                if let Some(distance) = shortest_paths.get(&to_node) {
                    actual_distance_matrix.insert((from_node, to_node), *distance);
                }
            }
        }

        let cost_matrix = actual_distance_matrix.clone();

        let config = AlgorithmConfig {
            n_candidate_segments: 500,
            search_radius_divisor: 4.0,
            k_top_candidates_to_consider: 2,
            m_candidates_to_evaluate: 2,
            penalty_factor: 4.0,
        };

        let route = generate_route(&graph, start_node, 10000.0, &top_candidates, &actual_distance_matrix, &cost_matrix, &config);
        assert!(route.is_some(), "generate_route should have returned a route");
        let route_path = route.unwrap();
        assert!(!route_path.is_empty());
        let total_dist: f64 = route_path.iter().map(|e| e.distance).sum();
        assert!(total_dist > 0.0);
    }

    #[test]
    fn test_highway_weighting_prefers_lower_weight_paths() {
        let mut graph = RouteGraph::new();
        let n1 = graph.add_node(Point { lat: 0.0, lon: 0.0 });
        let n2 = graph.add_node(Point { lat: 0.0, lon: 0.01 });
        let n3 = graph.add_node(Point { lat: 0.0, lon: 0.02 });
        let n4 = graph.add_node(Point { lat: 0.0, lon: 0.03 });

        // Path A (good): n1 -> n2. Trail.
        graph.add_edge(n1, n2, EdgeData { segment_id: 10, highway_type: HighwayType::Trail, path: vec![], distance: 100.0, weighted_distance: 100.0, ascent: 10.0, descent: 0.0, start_node: n1, end_node: n2 });
        // Path B (bad): n1 -> n3. PavedRoad.
        graph.add_edge(n1, n3, EdgeData { segment_id: 11, highway_type: HighwayType::PavedRoad, path: vec![], distance: 100.0, weighted_distance: 200.0, ascent: 10.0, descent: 0.0, start_node: n1, end_node: n3 });

        // Common path to end: n2 -> n4 and n3 -> n4
        graph.add_edge(n2, n4, EdgeData { segment_id: 12, highway_type: HighwayType::Trail, path: vec![], distance: 50.0, weighted_distance: 50.0, ascent: 5.0, descent: 0.0, start_node: n2, end_node: n4 });
        graph.add_edge(n3, n4, EdgeData { segment_id: 13, highway_type: HighwayType::Trail, path: vec![], distance: 50.0, weighted_distance: 50.0, ascent: 5.0, descent: 0.0, start_node: n3, end_node: n4 });

        // Add reverse edges
        graph.add_edge(n2, n1, EdgeData { segment_id: 10, highway_type: HighwayType::Trail, path: vec![], distance: 100.0, weighted_distance: 100.0, ascent: 0.0, descent: 10.0, start_node: n2, end_node: n1 });
        graph.add_edge(n3, n1, EdgeData { segment_id: 11, highway_type: HighwayType::PavedRoad, path: vec![], distance: 100.0, weighted_distance: 200.0, ascent: 0.0, descent: 10.0, start_node: n3, end_node: n1 });
        graph.add_edge(n4, n2, EdgeData { segment_id: 12, highway_type: HighwayType::Trail, path: vec![], distance: 50.0, weighted_distance: 50.0, ascent: 0.0, descent: 5.0, start_node: n4, end_node: n2 });
        graph.add_edge(n4, n3, EdgeData { segment_id: 13, highway_type: HighwayType::Trail, path: vec![], distance: 50.0, weighted_distance: 50.0, ascent: 0.0, descent: 5.0, start_node: n4, end_node: n3 });

        let start_node = n1;
        let top_candidates = vec![
             CandidateSegment { start_node: n2, end_node: n4, distance: 50.0, ascent: 5.0 },
        ];

        let mut key_nodes: HashSet<NodeIndex> = top_candidates.iter().flat_map(|s| [s.start_node, s.end_node]).collect();
        key_nodes.insert(start_node);
        let key_nodes_vec: Vec<NodeIndex> = key_nodes.into_iter().collect();

        let mut actual_distance_matrix = HashMap::new();
        let mut cost_matrix = HashMap::new();
        for &from_node in &key_nodes_vec {
            let shortest_paths_actual = dijkstra(&graph, from_node, None, |e| e.weight().distance);
            let shortest_paths_cost = dijkstra(&graph, from_node, None, |e| e.weight().weighted_distance);
            for &to_node in &key_nodes_vec {
                if let Some(distance) = shortest_paths_actual.get(&to_node) {
                    actual_distance_matrix.insert((from_node, to_node), *distance);
                }
                if let Some(distance) = shortest_paths_cost.get(&to_node) {
                    cost_matrix.insert((from_node, to_node), *distance);
                }
            }
        }

        let config = AlgorithmConfig {
            n_candidate_segments: 100,
            search_radius_divisor: 4.0,
            k_top_candidates_to_consider: 1,
            m_candidates_to_evaluate: 1,
            penalty_factor: 1.0,
        };

        let route = generate_route(&graph, start_node, 1000.0, &top_candidates, &actual_distance_matrix, &cost_matrix, &config).unwrap();

        // The tour should be n1 -> n2 -> n4 -> n1.
        // We check that the good path (segment 10) is in the route, and the bad one (11) is not.
        assert!(route.iter().any(|seg| seg.segment_id == 10));
        assert!(!route.iter().any(|seg| seg.segment_id == 11));
    }

    #[test]
    fn test_m_from_k_candidate_selection() {
        let (mut graph, start_node) = create_test_graph();
        let n1 = start_node;
        let n2 = graph.node_indices().nth(1).unwrap();
        let n3 = graph.node_indices().nth(2).unwrap();
        let n4 = graph.node_indices().nth(3).unwrap();

        // good_candidate has a better insertion cost, but is ranked lower than bad_candidate
        let good_candidate = CandidateSegment { start_node: n1, end_node: n2, distance: 1000.0, ascent: 10.0 };
        let bad_candidate = CandidateSegment { start_node: n3, end_node: n4, distance: 1000.0, ascent: 30.0 };

        let top_candidates = vec![bad_candidate.clone(), good_candidate.clone()];

        // Make the bad candidate have a high cost
        graph.edge_weight_mut(graph.find_edge(n1, n3).unwrap()).unwrap().weighted_distance = 10000.0;

        let mut key_nodes: HashSet<NodeIndex> = top_candidates.iter().flat_map(|s| [s.start_node, s.end_node]).collect();
        key_nodes.insert(start_node);
        let key_nodes_vec: Vec<NodeIndex> = key_nodes.into_iter().collect();

        let mut actual_distance_matrix = HashMap::new();
        for &from_node in &key_nodes_vec {
            let shortest_paths = dijkstra(&graph, from_node, None, |e| e.weight().distance);
            for &to_node in &key_nodes_vec {
                if let Some(distance) = shortest_paths.get(&to_node) {
                    actual_distance_matrix.insert((from_node, to_node), *distance);
                }
            }
        }

        let mut cost_matrix = HashMap::new();
        for &from_node in &key_nodes_vec {
            let shortest_paths = dijkstra(&graph, from_node, None, |e| e.weight().weighted_distance);
            for &to_node in &key_nodes_vec {
                if let Some(distance) = shortest_paths.get(&to_node) {
                    cost_matrix.insert((from_node, to_node), *distance);
                }
            }
        }

        let config = AlgorithmConfig {
            n_candidate_segments: 500,
            search_radius_divisor: 4.0,
            k_top_candidates_to_consider: 2, // K=2, so both are considered
            m_candidates_to_evaluate: 2, // M=2, so both are evaluated
            penalty_factor: 1.0,
        };

        let route = generate_route(&graph, start_node, 20000.0, &top_candidates, &actual_distance_matrix, &cost_matrix, &config).unwrap();

        // The route should contain the good candidate's segment (1->2, segment 1),
        // because it has a much lower insertion cost, even though the bad candidate is ranked higher.
        assert!(route.iter().any(|seg| seg.segment_id == 1));
    }
}
