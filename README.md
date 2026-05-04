*This project has been created as part of the 42 curriculum by joesanto.*

---

# Fly-In Drones — Pathfinding & Simulation Engine

## Description

Fly-In Drones is a simulation project that models the movement of autonomous drones through a constrained network of zones. Each zone has a type (`normal`, `restricted`, `priority`, `blocked`), a maximum drone capacity, and a position on a 2D grid. Drones must travel from a start hub to an end hub while respecting zone capacities, traversal times, multi-drone congestion, and pathfinding constraints.

The project includes:

- A full parser for the Fly-In map format
- A graph-based representation of the environment
- A pathfinding engine computing two shortest loopless paths (Yen's algorithm, K=2)
- A multi-agent simulation engine with congestion, delays, and zone-based cooldowns
- A visual renderer displaying the map and drone positions in real time
- A logging system outputting movements in the required format

The goal is to simulate realistic drone traffic while respecting all constraints of the Fly-In specification.

---

## Instructions

### Installation

Install all Python dependencies using the provided Makefile:

```bash
make install
```

### Run the simulation

```bash
make run
```

### Debug mode

Runs the program under Python's built-in debugger (`pdb`):

```bash
make debug
```

### Clean temporary files

Removes `__pycache__`, `.mypy_cache`, and other temporary artifacts:

```bash
make clean
```

### Linting

```bash
make lint
make lint-strict
```

---

## Algorithm Choices & Implementation Strategy

### 1. Graph Construction

The map file is parsed into a dictionary of `Zone` objects linked by bidirectional connections. Each zone stores its neighbors directly, forming an adjacency-list graph. Zone metadata includes `zone_type`, `max_drones`, `color`, and 2D coordinates.

### 2. Pathfinding

The engine uses **A\*** for shortest-path computation, with Manhattan distance as the admissible heuristic. Movement costs vary by zone type: `blocked` zones are impassable, `restricted` zones cost 2, and `normal`/`priority` zones cost 1.

On top of A\*, **Yen's algorithm (K=2)** computes the two best loopless shortest paths. This was chosen over naive alternatives because:

- Reversing neighbor order does not guarantee a distinct path
- Temporarily removing edges breaks valid paths in dense graphs
- Yen's algorithm rigorously guarantees the next-best path without cycles

Having two candidate paths enables the simulation to route drones across separate corridors, reducing congestion.

### 3. Multi-Agent Simulation

The simulation handles multiple drones simultaneously with the following mechanisms:

- **Zone capacity limits** — drones wait if the next zone is at capacity
- **Zone-based cooldowns** — restricted zones impose a traversal delay shared by all drones in that zone, ensuring correct timing regardless of arrival order
- **Front-to-back ordering** — drones are processed from the front of the path each turn, preventing phantom blocking caused by processing order
- **Staggered departures** — drones leave the start hub one turn apart to avoid immediate congestion
- **Drone removal on delivery** — drones are removed from the simulation once they reach the end hub

The key design decision was zone-based rather than per-drone cooldowns. This correctly models the physical constraint that a restricted zone takes a fixed number of turns to cross, regardless of which drone is traversing it.

### 4. Logging System

Each turn outputs only the drones that moved, in the required format:

```
D1-roof1 D2-corridorA
D1-roof2 D2-tunnelB
D1-goal D2-goal
```

Movement through a restricted zone prints the connection name rather than the zone name, as specified.

---

## Visual Representation

The simulation includes a grid-based renderer that updates every turn and provides:

- All zones drawn as labeled cells at their 2D coordinates
- Connections rendered between adjacent zones
- Drone positions shown as `D` markers animating across the grid
- Real-time updates reflecting congestion and traversal delays

These visual features enhance the user experience by making pathfinding behavior intuitive, drone flow easy to follow, congestion immediately visible, and debugging significantly simpler than reading raw log output alone.

---

## Resources

### Documentation & References

- [A* Pathfinding — Red Blob Games](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [Yen's K-Shortest Paths Algorithm — Original paper by Jin Y. Yen (1971)](https://doi.org/10.1287/mnsc.17.11.712)

### AI Usage

AI assistance was used for clarifying algorithmic approaches (A\*, Yen's algorithm, simulation design), debugging type-checking issues with `mypy` and writing documentation including this README.

All implementation decisions, final code, and architecture were written and validated by the project authors.
