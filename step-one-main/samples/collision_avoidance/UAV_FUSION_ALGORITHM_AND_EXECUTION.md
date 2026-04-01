10# UAV fusion path planning: algorithm and code execution
0
This document describes the **two-layer planner** implemented in `UAVWaypointMovement` (THE ONE / STEP-ONE simulator), based on:

**He, Hou & Wang (2024).** “A new method for unmanned aerial vehicle path planning in complex environments.” *Scientific Reports* 14:9257.

---

## Part 1 — The algorithm (what the maths does)

### Big picture

The UAV needs a **global** route through a cluttered map and a **local** controller that tracks that route while dodging obstacles in real time. The paper combines:

1. **Layer 1 — improved A\*** on a grid → coarse but obstacle-aware route.  
2. **Bresenham compression** → keep only “corner” waypoints (key nodes) along that route.  
3. **Layer 2 — improved DWA (Dynamic Window Approach)** → each short step picks a velocity/heading that scores well for: aiming at the next key node, preferred speed, clearance from obstacles, and staying near the global segment.  
4. **Fusion** → DWA always chases the **current key node**; when that is reached, the planner advances to the **next** key node until the final POI/goal is reached.

So: **A\*** answers “roughly which corridor of cells should we use?” and **DWA** answers “how do we move this 0.1 s slice without hitting anything?”

### Layer 1 — grid A\* with an adaptive heuristic

- The world is divided into cells of side `gridCellM` (metres). Each cell is **free** or **blocked** (from WKT obstacles rasterised onto the grid).  
- **A\*** searches from start cell to goal cell using 8-connectivity (orthogonal + diagonal moves; diagonal moves are blocked if they would cut a corner through a wall).  
- **Cost `g`** accumulates Euclidean-ish step costs scaled by `gridCellM`.  
- **Heuristic `h`** is based on **Manhattan distance** in metres, multiplied by a factor  
  `1 + sigmoid(ξ)`  
  where **ξ** is the **fraction of cells that are obstacles** inside the axis-aligned rectangle between the current cell and the goal. Many obstacles ahead → stronger heuristic → search becomes greedier (paper Eqs. 7–9).  
- **Neighbour clipping** reduces which neighbours are expanded from a node so redundant “zig-zag” expansions are trimmed (paper Fig. 1).

**Output of Layer 1** is a **polyline in world coordinates** (cell centres along the path, plus the exact goal).

### Bresenham key nodes

The raw A\* path has many tiny steps. The code **compresses** it:

- Walk along the path from a point `p0` and try to draw a straight line to `p2`, `p3`, … as far as possible.  
- **Bresenham ray casting** on the grid checks whether that line crosses any blocked cell.  
- The last **still-visible** point becomes the next **key node**; repeat until the goal.

So **key nodes** are the **break points** where you cannot shortcut in a straight line without cutting through an obstacle.

### Layer 2 — improved DWA

At each control instant, DWA does **not** replan full A\*; it only plans a **short step** toward the **current key node** (sub-goal).

- Sample several **headings** fanning around the bearing to the sub-goal, and several **speeds** in `[speedMin, speedMax]`.  
- For each candidate end point of a short step:  
  - **Heading score** — align motion with direction to sub-goal.  
  - **Velocity score** — prefer higher cruise speed.  
  - **Obstacle score** — prefer points farther from obstacles (nearest distance to WKT geometry after rasterisation).  
  - **Path-following score** — stay close to the straight segment from current position to sub-goal (ties local motion to the global plan).

**Adaptive weights** (`α, β, λ, η` in the paper): when the UAV is **far** from obstacles, **path following** dominates (`η` high). N closer than `distAlert` → **obstacle term** gets more weight. N closer than `distRisk` → **strong avoidance** mode.

The candidate with the **best weighted sum** wins; the UAV moves one step toward that point.

### Fusion summary

| Stage | Role |
|--------|------|
| A\* | Global feasible cell path |
| Key nodes | Sparse waypoints along that path |
| DWA | Local motion toward current key node, reactive avoidance |

---

## Part 2 — How the code runs inside THE ONE / STEP-ONE

### Settings namespaces (important)

- **Group settings** (e.g. `Group1.movementModel`, `Group1.speed`) are read with a `Group1`-scoped `Settings` object.  
- **All UAV-specific parameters** (`spawn`, `target`, `gridCellM`, WKT path, DWA options, etc.) live under the prefix **`UAVWaypointMovement.`** in the text file. The constructor creates `new Settings("UAVWaypointMovement")` to read them.  
- **World size** still comes from `MovementModel.worldSize` (e.g. `500, 500`).

### Class loading and prototype host

1. **Scenario loads** → for each group, ONE instantiates a **movement model prototype** with `Settings` scoped to `Group1` (etc.).  
2. `UAVWaypointMovement(Settings s)` runs:  
   - `super(s)` — reads `Group` speed/wait from `Group1`.  
   - Reads **mission + planner** settings from **`UAVWaypointMovement.*`**.  
   - Requires **`spawn`** and **`target`** (CSV `x, y` in metres).  
   - Computes `gridW`, `gridH`, and publishes an **empty** planning-grid snapshot so the GUI can draw grid lines even before the first sim step.  
3. **Other hosts** get **`replicate()`** copies sharing the same parameters; **obstacle grid** is **static** and built once.

### `getInitialLocation()` — once per host when simulation places it

1. **First UAV only (guard `obstaclesBuilt`)**:  
   - `buildObstacleWorld()` allocates the shared boolean grid, clears WKT disc/segment lists, optionally loads **`obstacleWktFile`** (POINT + LINESTRING parsed by `WktObstacleParser`), rasterises onto cells, updates `PlanningGridSnapshot` with true obstacles.  
2. Places the UAV at **`spawn`** (optionally snapped to nearest **free** cell centre).  
3. **`buildMissionPoiTour`**: optional lattice of POIs (`poiGridCols` × `poiGridRows`), then **target** last; nearest-neighbour order through lattice points.  
4. **`planToNextPoi()`** → run **A\*** to current tour point, **Bresenham** → **`keyNodes`**, `keyIndex = 0`.

### Simulation loop — `getPath()` every movement tick

ONE repeatedly asks the movement model for the next **`Path`** (here: a single waypoint and a sampled speed).

1. If `keyNodes` is empty, **`planToNextPoi()`** again.  
2. **`subGoal = keyNodes.get(keyIndex)`**.  
3. **`dwaStep(uavPos, subGoal)`** — one DWA iteration → candidate next world position (clamped, not inside obstacle cell).  
4. If within **`gridCellM`** of `subGoal`, snap to sub-goal and **`keyIndex++`**. If all key nodes done → **arrived at current tour waypoint** → advance **`poiTour`**, rebuild A\*/key nodes for next leg (or restart lap with a new NN tour).  
5. Build **`Path`**: one waypoint at updated **`uavPos`**, speed from **`sampleSpeed()`**.

So **one call to `getPath()` ≡ one DWA step** (plus occasionally replanning A\* when changing sub-goals or POIs).

### `nextPathAvailable()` — dwell at waypoints

When ONE considers the UAV to have “arrived”, it calls **`nextPathAvailable()`**, which returns a random time in **`[dwellMin, dwellMax]`** so the host pauses (hovering) before the next **`getPath()`**.

### WKT obstacles

- **`obstacles.wkt`**: one `POINT` or `LINESTRING` per line (see sample file).  
- **POINT**: disk of radius **`pointObstacleRadius`** rasterised to cells; DWA distance uses a circle.  
- **LINESTRING**: polyline with half-width **`lineObstacleHalfWidth`** rasterised; DWA distance uses buffered segments.  
- Empty **`obstacleWktFile`** → no blocked cells (free world).

### Optional GUI grid

- **`UAVWaypointMovement.showPlanningGrid = true`** — each **`PlayField` paint** calls **`UavPlanningGridRenderer.renderIfEnabled`**, which draws grid lines and blocked cells from **`PlanningGridSnapshot`** (not the overlay list, so **Clear overlays** in the menu does not remove it).  
- Set to **`false`** to hide.

---

## Part 3 — File map (where things live)

| Piece | Location |
|--------|-----------|
| Movement model + A\*, DWA, mission | `the-one/src/movement/UAVWaypointMovement.java` |
| WKT parsing | `the-one/src/movement/WktObstacleParser.java` |
| Grid drawing | `the-one/src/gui/uavviz/PlanningGridOverlayGraphic.java`, `UavPlanningGridRenderer.java`; hook in `PlayField.java` |
| Example scenario | `step-one-main/samples/collision_avoidance/uav_citymap_settings.txt` + `obstacles.wkt` |

---

## Part 4 — Quick settings checklist

- **`UAVWaypointMovement.spawn`** / **`target`** — required.  
- **`obstacleWktFile`** — optional path (relative to process working directory).  
- **`gridCellM`** — cell size for A\* and rasterisation.  
- **`dwaSteps`, `distAlert`, `distRisk`** — DWA sampling and safety bands.  
- **`poiGridCols`, `poiGridRows`** — optional intermediate stops before **`target`**.  
- **`showPlanningGrid`** — GUI grid on/off.

Run the STEP-ONE GUI from the directory where paths like `samples/collision_avoidance/obstacles.wkt` resolve correctly (same as when you pass the settings file to Gradle/`main`).
