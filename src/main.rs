use osmpbf::{Element, ElementReader};
use std::error::Error;

#[derive(Debug)]
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

fn main() -> Result<(), Box<dyn Error>> {
    let path = "data/oxfordshire-250907.osm.pbf";
    let reader = ElementReader::from_path(path)?;

    let mut nodes = 0;
    let mut ways = 0;
    let mut valid_ways = 0;
    let mut relations = 0;
    let mut parking_locations = Vec::new();

    reader.for_each(|element| match element {
        Element::Node(node) => {
            nodes += 1;
            if node
                .tags()
                .any(|(key, value)| key == "amenity" && value == "parking")
            {
                parking_locations.push(Point {
                    lat: node.lat(),
                    lon: node.lon(),
                });
            }
        }
        Element::DenseNode(node) => {
            nodes += 1;
            if node
                .tags()
                .any(|(key, value)| key == "amenity" && value == "parking")
            {
                parking_locations.push(Point {
                    lat: node.lat(),
                    lon: node.lon(),
                });
            }
        }
        Element::Way(way) => {
            ways += 1;
            if is_valid_way(way.tags()) {
                valid_ways += 1;
            }
        }
        Element::Relation(_) => relations += 1,
    })?;

    println!("Successfully parsed PBF file.");
    println!("Nodes: {}", nodes);
    println!("Ways: {} ({} valid for routing)", ways, valid_ways);
    println!("Relations: {}", relations);
    println!("Parking Locations (Nodes): {}", parking_locations.len());

    Ok(())
}
