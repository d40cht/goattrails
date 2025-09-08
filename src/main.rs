use osmpbf::{Element, ElementReader};
use std::collections::HashMap;
use std::error::Error;

#[derive(Debug, Clone, Copy)]
struct Point {
    lat: f64,
    lon: f64,
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

    // --- Pass 2: Identify ways and find parking ---
    println!("Starting pass 2: Processing ways and amenities...");
    let reader2 = ElementReader::from_path(path)?;
    let mut ways = 0;
    let mut valid_ways = 0;
    let mut relations = 0;
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
            ways += 1;
            if is_valid_way(way.tags()) {
                valid_ways += 1;
            }

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
        Element::Relation(_) => relations += 1,
    })?;

    println!("
Successfully parsed PBF file.");
    println!("Total Nodes: {}", node_coords.len());
    println!("Total Ways: {} ({} valid for routing)", ways, valid_ways);
    println!("Total Relations: {}", relations);
    println!(
        "Found {} parking locations ({} from nodes, {} from ways).",
        parking_locations.len(),
        parking_nodes,
        parking_ways
    );

    Ok(())
}
