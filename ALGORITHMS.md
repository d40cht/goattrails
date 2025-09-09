# Route Generation Algorithm

The application uses a two-level process to find optimal routes: an outer loop for generating multiple candidates, and a core inner algorithm for generating a single route. The parameters for this process are controlled by `config.toml`.

## 1. Overall Process (Outer Loop)

The main application logic in `src/main.rs` orchestrates the high-level route finding strategy:

1.  **Configuration:** The application loads its parameters from a `config.toml` file. This includes the number of routes to generate (`X`), the number of top routes to display (`Y`), and the target distance.
2.  **Graph Construction:** A graph of the local area is built from OpenStreetMap and SRTM data.
3.  **Start Point Selection:** A random parking location is chosen as the general starting area, and the nearest node in the graph is selected as the precise starting point for all route candidates.
4.  **Candidate Generation Loop:** The application runs an outer loop `X` times (`route_candidates_to_generate`). In each iteration, it calls the core route generation algorithm to produce one complete route candidate.
5.  **Ranking and Selection:** After generating `X` routes, the application filters out any that failed to generate. The successful routes are then sorted in descending order based on their total ascent. The top `Y` routes (`top_routes_to_display`) are selected from this sorted list.
6.  **Export:** The top `Y` routes are passed to the map exporter, which creates an HTML file (`vis/final_route.html`) displaying each route as a colored polyline with a popup containing its statistics.

---

## 2. Core Route Generation Algorithm (Greedy Insertion Heuristic)

The core algorithm, located in `src/route_generator.rs`, uses a constructive heuristic to build a high-ascent circular route. It builds a "tour" of valuable segments by iteratively inserting them into a route, keeping the total distance within the configured budget.

### 2.1. Pre-computation and Candidate Selection

Before building the route, the algorithm performs two pre-computation steps:

1.  **Candidate Segment Identification:** The algorithm first creates a pool of desirable segments.
    a.  It filters all edges in the graph to find those that are within a radius of `target_distance / 2.0` from the `start_node`.
    b.  From this subset, it selects the top 200 segments based on a value heuristic of `ascent / distance`. Only segments with more than 5 meters of ascent are considered.

2.  **All-Pairs Shortest Path (APSP) Matrix:** To quickly evaluate the cost of connecting segments, the algorithm pre-calculates the shortest travel distance between all "key nodes" (the start and end nodes of all candidate segments, plus the main `start_node`). This is done by running Dijkstra's algorithm from each key node and storing the results in a `HashMap` for instant lookup.

### 2.2. Greedy Insertion Loop

The route is built iteratively by adding one segment at a time from the candidate pool.

1.  **Stochastic Candidate Selection:** At each step of the loop, instead of processing segments in a fixed order, the algorithm considers the top-k (currently k=5) best-valued segments remaining in the candidate pool. It randomly selects one of these 5 candidates to be the next segment to attempt to insert. This introduces variability, allowing the algorithm to generate different routes on each run.

2.  **Cheapest Insertion:** For the selected candidate segment, the algorithm evaluates all possible insertion points in the current tour. It determines the "cheapest" insertion point—the one that adds the minimum extra travel distance required to connect the new segment into the tour.

3.  **Insertion and Budget Check:** If the cheapest insertion results in a new total route distance that is still within the `target_distance`, the segment is permanently added to the tour at that position. If not, the candidate is discarded, and the algorithm moves to the next iteration.

4.  **Termination:** The loop continues until the pool of remaining candidates is empty, or until a full pass over the candidates results in no successful insertions.

### 2.3. Final Path Reconstruction

The output of the greedy loop is an ordered list of high-value segments. The final step is to create a complete, traversable path.

1.  **Discouraging Reversals:** To prevent awkward "there-and-back" paths, a "discouraged list" is created containing the reverse edge of every segment in the final tour.
2.  **Stitching with A*:** The A* algorithm is used to find the shortest path to connect the segments in order: from the main `start_node` to the start of the first segment, from the end of the first to the start of the second, and so on, until the final path from the end of the last segment back to the main `start_node` is calculated.
3.  **Penalty System:** The A* cost function is modified to heavily penalize (by 4x) any edge that is in the "discouraged list". This ensures the algorithm will find an alternative to an immediate reversal if one exists, but can still use it as a last resort if it's the only way to connect the tour (e.g., over a bridge).

The final result is a single, continuous path of `EdgeData` structs representing the complete high-ascent route.
