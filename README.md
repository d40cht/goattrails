# GoatTrails 🐐

**GoatTrails** is a route discovery tool for runners and hikers looking to maximize elevation gain in areas with limited topography. It finds challenging circular routes by analyzing path networks and elevation data, mimicking how a goat might find the steepest path up a mountain.

This project is designed to help athletes train for mountainous ultra-marathons by finding the most "mountain-like" routes available in flatter regions.

---

## Features

* **Novel Route Generation:** Discovers new route combinations from raw OpenStreetMap data, rather than relying on pre-defined segments.
* **Ascent Optimization:** Finds routes that maximize total elevation gain for a user-specified target distance ($N$ km).
* **Smart Filtering:** Prioritizes runnable trails, footpaths, and quiet country lanes while filtering out motorways and busy main roads.
* **Visualization Export:** Generates self-contained HTML files to visualize the recommended route on an interactive map using Leaflet.js.

---

## Technical Architecture

The tool works by processing geospatial data through a multi-phase pipeline:

1.  **Data Ingestion:** Parses OpenStreetMap (`.pbf`) data to extract all potential paths, tracks, and roads.
2.  **Graph Construction:** Builds a graph representation of the area using `petgraph`. Nodes in the graph represent intersections, and edges represent the path segments connecting them.
3.  **Elevation Enrichment:** Uses SRTM digital elevation model (DEM) data to calculate the precise distance, ascent, and descent for every edge in the graph.
4.  **Route Optimization:** Implements search algorithms (e.g., constrained DFS, genetic algorithms) to find simple cycles that best match the user's criteria (target distance and maximum ascent).
5.  **Export:** Outputs the final route coordinates to a simple HTML/JavaScript viewer.

---

## Development Status

This project is currently in development. The core logic for graph construction and route optimization is being built.

### Development Roadmap

* **Phase 1:** OSM data parsing and `petgraph` model construction.
* **Phase 2:** SRTM data integration and edge weighting (distance, ascent).
* **Phase 3:** Implementation of cycle-finding and optimization algorithms.
* **Phase 4:** HTML visualization export feature.

---

## Installation & Usage (Planned)

Clone the repository and build using Cargo:

```bash
git clone [https://github.com/your-username/goattrails.git](https://github.com/your-username/goattrails.git)
cd goattrails
cargo build --release
