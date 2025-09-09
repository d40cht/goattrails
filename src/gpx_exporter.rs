use crate::EdgeData;
use geo::Point;
use gpx::{write, Gpx, Track, TrackSegment, Waypoint};
use std::fs::File;
use std::io::BufWriter;

pub fn export_gpx(
    route: &[EdgeData],
    route_name: &str,
    route_idx: usize,
) -> Result<(), std::io::Error> {
    let mut gpx = Gpx::default();
    gpx.version = gpx::GpxVersion::Gpx11;

    let mut track = Track::new();
    track.name = Some(format!("{} #{}", route_name, route_idx + 1));

    let mut segment = TrackSegment::new();

    let all_points: Vec<Waypoint> = route
        .iter()
        .flat_map(|edge| edge.path.iter())
        .map(|point| Waypoint::new(Point::new(point.lon, point.lat)))
        .collect();

    segment.points = all_points;
    track.segments.push(segment);
    gpx.tracks.push(track);

    let filename = format!("vis/{}_{}.gpx", route_name, route_idx);
    let file = File::create(&filename)?;
    let buf = BufWriter::new(file);

    write(&gpx, buf).map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    println!("-> Saved GPX route to {}", filename);

    Ok(())
}
