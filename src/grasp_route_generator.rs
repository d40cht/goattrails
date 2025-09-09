use crate::{AlgorithmConfig, CandidateSegment, EdgeData, RouteGraph};
use indicatif::{ProgressBar, ProgressStyle};
use petgraph::algo::astar;
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use rand::seq::SliceRandom;
use std::collections::{HashMap, HashSet};

const RCL_SIZE: usize = 10;
const K_TOP_CANDIDATES: usize = 5;

struct ScoredCandidate {
    score: f64,
    candidate: CandidateSegment,
    insertion_index: usize,
    new_properties: (f64, f64), // (distance, ascent)
}

pub fn generate_route_grasp(
    graph: &RouteGraph,
    start_node: NodeIndex,
    target_distance: f64,
    top_candidates: &[CandidateSegment],
    actual_distance_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
    cost_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
    config: &AlgorithmConfig,
) -> Option<Vec<EdgeData>> {
    println!("Starting route generation with GRASP heuristic...");

    let mut tour: Vec<CandidateSegment> = Vec::new();
    let mut current_properties = (0.0, 0.0);
    let mut remaining_candidates = top_candidates.to_vec();
    let mut rng = rand::thread_rng();

    let bar = ProgressBar::new(remaining_candidates.len() as u64);
    bar.set_style(
        ProgressStyle::default_bar()
            .template(
                "{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta}) {msg}",
            )
            .unwrap()
            .progress_chars("#>-"),
    );

    // Handle the first segment separately, as there's no tour to insert into yet.
    let first_candidate_opt = remaining_candidates
        .iter()
        .filter(|c| c.distance <= target_distance)
        .max_by(|a, b| a.ascent.partial_cmp(&b.ascent).unwrap())
        .cloned();

    if let Some(first_candidate) = first_candidate_opt {
        tour.push(first_candidate.clone());
        remaining_candidates.retain(|c| c.start_node != first_candidate.start_node || c.end_node != first_candidate.end_node);
        if let Some(props) = calculate_tour_properties_from_segments(start_node, &tour, actual_distance_matrix) {
            current_properties = props;
        }
        bar.inc(1);
    } else {
        println!("No suitable first candidate found.");
        return None;
    }

    loop {
        if remaining_candidates.is_empty() {
            println!("No candidates left.");
            break;
        }
        bar.inc(1);

        let mut scored_candidates: Vec<ScoredCandidate> = Vec::new();

        for candidate in &remaining_candidates {
            if let Some(best_insertion) =
                find_cheapest_insertion(candidate, &tour, start_node, target_distance, actual_distance_matrix, cost_matrix)
            {
                let (insertion_index, new_total_distance, new_total_ascent) = best_insertion;
                let insertion_cost = new_total_distance - current_properties.0;

                if insertion_cost > 0.0 {
                    let score = candidate.ascent / insertion_cost;
                    scored_candidates.push(ScoredCandidate {
                        score,
                        candidate: candidate.clone(),
                        insertion_index,
                        new_properties: (new_total_distance, new_total_ascent),
                    });
                }
            }
        }

        if scored_candidates.is_empty() {
            println!("No valid insertions found in this iteration.");
            break;
        }

        scored_candidates.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());

        let rcl = &scored_candidates[..std::cmp::min(RCL_SIZE, scored_candidates.len())];
        if rcl.is_empty() {
            println!("RCL is empty, cannot proceed.");
            break;
        }

        let pool_size = std::cmp::min(K_TOP_CANDIDATES, rcl.len());
        let chosen_one = rcl[..pool_size].choose(&mut rng).unwrap();

        tour.insert(chosen_one.insertion_index, chosen_one.candidate.clone());
        current_properties = chosen_one.new_properties;
        remaining_candidates.retain(|c| c.start_node != chosen_one.candidate.start_node || c.end_node != chosen_one.candidate.end_node);

        bar.set_message(format!(
            "Dist: {:.2}km, Ascent: {:.1}m",
            current_properties.0 / 1000.0,
            current_properties.1
        ));
    }
    bar.finish();

    if tour.is_empty() {
        eprintln!("Could not construct a route with the given constraints.");
        return None;
    }

    println!("Final tour has {} segments.", tour.len());
    println!(
        "Final route estimate: Distance = {:.2}km, Ascent = {:.2}m",
        current_properties.0 / 1000.0,
        current_properties.1
    );

    println!("\nReconstructing final route path...");
    let final_path_nodes =
        reconstruct_final_path(graph, start_node, &tour, config.penalty_factor);

    if final_path_nodes.is_empty() {
        return None;
    }

    println!("Final path has {} nodes.", final_path_nodes.len());
    Some(build_edge_data_path(graph, &final_path_nodes))
}

fn find_cheapest_insertion(
    candidate: &CandidateSegment,
    tour: &[CandidateSegment],
    start_node: NodeIndex,
    target_distance: f64,
    actual_distance_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
    cost_matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
) -> Option<(usize, f64, f64)> { // (insertion_index, new_dist, new_ascent)
    let mut best_insertion: Option<(usize, f64, f64)> = None;
    let mut min_cost = f64::INFINITY;

    let (current_cost, _) = calculate_tour_properties_from_segments(start_node, tour, cost_matrix).unwrap_or((0.0, 0.0));

    for i in 0..=tour.len() {
        let mut temp_tour = tour.to_vec();
        temp_tour.insert(i, candidate.clone());

        if let Some((new_dist, new_ascent)) =
            calculate_tour_properties_from_segments(start_node, &temp_tour, actual_distance_matrix)
        {
            if new_dist <= target_distance {
                if let Some((new_cost, _)) =
                    calculate_tour_properties_from_segments(start_node, &temp_tour, cost_matrix)
                {
                    let cost = new_cost - current_cost;
                    if cost < min_cost {
                        min_cost = cost;
                        best_insertion = Some((i, new_dist, new_ascent));
                    }
                }
            }
        }
    }
    best_insertion
}

fn calculate_tour_properties_from_segments(
    start_node: NodeIndex,
    tour: &[CandidateSegment],
    matrix: &HashMap<(NodeIndex, NodeIndex), f64>,
) -> Option<(f64, f64)> {
    if tour.is_empty() {
        return Some((0.0, 0.0));
    }
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

fn reconstruct_final_path(
    graph: &RouteGraph,
    start_node: NodeIndex,
    tour: &[CandidateSegment],
    penalty_factor: f64,
) -> Vec<NodeIndex> {
    let discouraged_edges: HashSet<(NodeIndex, NodeIndex)> =
        tour.iter().map(|segment| (segment.end_node, segment.start_node)).collect();

    let mut final_path_nodes = vec![start_node];
    let mut current_node = start_node;
    let path_bar = ProgressBar::new(tour.len() as u64 + 1);
    path_bar.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} Reconstructing path... [{bar:40.cyan/blue}] {pos}/{len}")
            .unwrap()
            .progress_chars("#>-"),
    );

    for segment in tour {
        path_bar.inc(1);
        if let Some((_, path)) = astar(
            graph,
            current_node,
            |n| n == segment.start_node,
            |e| {
                if discouraged_edges.contains(&(e.source(), e.target())) {
                    e.weight().distance * penalty_factor
                } else {
                    e.weight().distance
                }
            },
            |_| 0.0,
        ) {
            final_path_nodes.extend(&path[1..]);
        } else {
            eprintln!("Could not find path between {:?} and {:?}", current_node, segment.start_node);
            return Vec::new();
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
                e.weight().distance * penalty_factor
            } else {
                e.weight().distance
            }
        },
        |_| 0.0,
    ) {
        final_path_nodes.extend(&path[1..]);
    } else {
        eprintln!("Could not find path back to start node from {:?}", current_node);
        return Vec::new();
    }
    path_bar.finish();
    final_path_nodes
}

fn build_edge_data_path(graph: &RouteGraph, nodes: &[NodeIndex]) -> Vec<EdgeData> {
    let mut path = Vec::new();
    for i in 0..nodes.len() - 1 {
        if let Some(edge_ref) = graph.find_edge(nodes[i], nodes[i + 1]) {
            path.push(graph.edge_weight(edge_ref).unwrap().clone());
        } else {
            eprintln!(
                "Error: No direct edge found between nodes {:?} and {:?}. This indicates a broken path.",
                nodes[i],
                nodes[i + 1]
            );
            return Vec::new();
        }
    }
    path
}
