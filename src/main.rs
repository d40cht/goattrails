use osmpbf::{Element, ElementReader};
use petgraph::graph::Graph;
use std::collections::HashMap;
use std::error::Error;

#[derive(Debug, Clone, Copy)]
struct Point {
    lat: f64,
    lon: f64,
}

#[derive(Debug, Clone)]
struct EdgeData {
    path: Vec<Point>,
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

// Calculate the centroid of a polygon (or a line).
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

fn main() -> Result<(), Box<dyn Error>> {
    let path = "data/oxfordshire-250907.osm.pbf";

    // --- Pass 1: Collect all node coordinates ---
    println!("Starting pass 1: Caching node coordinates...");
    let mut node_coords = HashMap::new();
    let reader1 = ElementReader::from_path(path)?;
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
    let reader2 = ElementReader::from_path(path)?;
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
    let reader3 = ElementReader::from_path(path)?;
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

    // Add intersection nodes to the graph
    for (osm_id, _) in &intersection_nodes {
        if let Some(point) = node_coords.get(osm_id) {
            let node_index = graph.add_node(*point);
            osm_id_to_node_index.insert(*osm_id, node_index);
        }
    }

    let reader4 = ElementReader::from_path(path)?;
    let mut ways = 0;
    let mut valid_ways = 0;

    reader4.for_each(|element| {
        if let Element::Way(way) = element {
            ways += 1;
            if is_valid_way(way.tags()) {
                valid_ways += 1;
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
                                EdgeData { path: current_path_segment.clone() },
                            );
                        }
                        last_intersection_node_id = Some(node_id);
                        current_path_segment = vec![*point];
                    }
                }
            }
        }
    })?;

    println!("Pass 4 complete.");
    println!("\n--- Graph Construction Summary ---");
    println!("Total Nodes in Graph: {}", graph.node_count());
    println!("Total Edges in Graph: {}", graph.edge_count());
    println!("Total Ways Scanned: {} ({} valid for routing)", ways, valid_ways);

    Ok(())
}
