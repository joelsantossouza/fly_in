from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Generator
import heapq
import time


@dataclass
class Zone:
    """
    Represents a zone (node) in the Fly-in map graph.

    Attributes:
        name (str): Unique identifier of the zone.
        x (int): X coordinate of the zone.
        y (int): Y coordinate of the zone.
        zone_type (str): Type of the zone (normal, blocked, restricted, priority).
        max_drones (int): Maximum number of drones allowed simultaneously.
        color (str | None): Optional color for visualization.
        connections (List[Zone]): Adjacent zones (neighbors in the graph).
    """
    name: str
    x: int
    y: int
    zone_type: str = "normal"
    max_drones: int = 1
    color: str | None = None
    connections: List["Zone"] = field(default_factory=list)

    def __hash__(self) -> int:
        return hash(self.name)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Zone):
            return False
        return self.name == other.name


@dataclass
class Drone:
    """
    Represent a Drone position in Fly-in map graph.

    Attributes:
        x, y: Bidimensional position on graph
    """
    x: int
    y: int


class PathFinder:
    """
    Handles all pathfinding logic for the Fly-in map.
    Uses A* with movement-cost rules based on zone types.
    """

    def __init__(self, map_obj: "Map") -> None:
        self.map: Map = map_obj

    # ---------------------------------------------------------
    # Movement cost based on zone type
    # ---------------------------------------------------------
    def movement_cost(self, zone: "Zone") -> float:
        if zone.zone_type == "blocked":
            return float("inf")
        if zone.zone_type == "restricted":
            return 2.0
        if zone.zone_type == "priority":
            return 1.0
        return 1.0  # normal

    # ---------------------------------------------------------
    # Heuristic: Manhattan distance (admissible)
    # ---------------------------------------------------------
    def heuristic(self, a: "Zone", b: "Zone") -> int:
        return abs(a.x - b.x) + abs(a.y - b.y)

    # ---------------------------------------------------------
    # A* algorithm
    # ---------------------------------------------------------
    def a_star(self, start: "Zone", goal: "Zone") -> List["Zone"]:
        open_set: List[Tuple[float, int, Zone]] = []
        counter: int = 0

        heapq.heappush(open_set, (0.0, counter, start))

        came_from: Dict[Zone, Zone] = {}
        g_score: Dict[Zone, float] = {start: 0.0}
        f_score: Dict[Zone, float] = {
            start: float(self.heuristic(start, goal))}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            for neighbor in current.connections:
                cost: float = self.movement_cost(neighbor)
                if cost == float("inf"):
                    continue

                tentative_g: float = g_score[current] + cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    priority_bonus: float = -0.1 if neighbor.zone_type == "priority" else 0.0

                    f_score[neighbor] = (
                        tentative_g
                        + float(self.heuristic(neighbor, goal))
                        + priority_bonus
                    )

                    counter += 1
                    heapq.heappush(
                        open_set, (f_score[neighbor], counter, neighbor))

        return []

    # ---------------------------------------------------------
    # Reconstruct path from A*
    # ---------------------------------------------------------

    def reconstruct_path(
        self,
        came_from: Dict["Zone", "Zone"],
        start: "Zone",
        goal: "Zone"
    ) -> List["Zone"]:
        path: List[Zone] = []
        current: Zone = goal

        while current in came_from:
            path.append(current)
            current = came_from[current]

        path.append(start)
        return list(reversed(path))

    # ---------------------------------------------------------
    # Public method to compute path
    # ---------------------------------------------------------
    def find_path(self) -> List["Zone"]:
        start_zone: Zone = self.map.zones[self.map.start]
        end_zone: Zone = self.map.zones[self.map.end]
        return self.a_star(start_zone, end_zone)

    # ---------------------------------------------------------
    # Optional: Convert path to movement directions
    # ---------------------------------------------------------
    def path_to_directions(self, path: List["Zone"]) -> List[str]:
        directions: List[str] = []

        for a, b in zip(path, path[1:]):
            if b.x > a.x:
                directions.append("RIGHT")
            elif b.x < a.x:
                directions.append("LEFT")
            elif b.y > a.y:
                directions.append("DOWN")
            elif b.y < a.y:
                directions.append("UP")

        return directions


class Map:
    """
    Manages the Fly-in map, including parsing input files and building the graph.

    Attributes:
        zones (Dict[str, Zone]): Dictionary of all zones indexed by name.
        connections (List[Tuple[str, str]]): Raw connections between zones.
        nb_drones (int): Number of drones in the simulation.
        start (str | None): Name of the start zone.
        end (str | None): Name of the end zone.
    """
    CELL_W: int = 5
    CELL_H: int = 5

    def __init__(self, filepath: str) -> None:
        """Initialize an empty map."""
        self.zones: Dict[str, Zone] = {}
        self.connections: List[Tuple[str, str]] = []
        self.drones: list[Drone] = []
        self.nb_drones: int = 0
        self.start: str | None = None
        self.end: str | None = None
        self.parse(filepath)

        zones_lst = list(self.zones.values())
        self.min_x = min(z.x for z in zones_lst)
        self.max_x = max(z.x for z in zones_lst)
        self.min_y = min(z.y for z in zones_lst)
        self.max_y = max(z.y for z in zones_lst)
        self.width = (self.max_x - self.min_x + 1) * self.CELL_W
        self.height = (self.max_y - self.min_y + 1) * self.CELL_H
        self.grid = [
            [" " for _ in range(self.width)]
            for _ in range(self.height)
        ]
        self.grid_fill()

        self.pathfinder: PathFinder = PathFinder(self)
        self.path: list[Zone] = self.pathfinder.find_path()

    def parse(self, filepath: str) -> None:
        """
        Parse a map file and build internal structures.

        Args:
            filepath (str): Path to the input file.

        Raises:
            ValueError: If the file format is invalid.
        """
        try:
            with open(filepath, "r") as file:
                for line_nb, line in enumerate(file, 1):
                    line = line.strip()

                    if not line or line.startswith("#"):
                        continue

                    if line.startswith("nb_drones:"):
                        self.nb_drones = self.parse_nb_drones(line)

                    elif line.startswith("start_hub:"):
                        zone = self.parse_zone(line, "start")
                        if self.start:
                            self.error(line_nb, "Multiple start_hub defined")
                        self.start = zone.name
                        self.zones[zone.name] = zone

                    elif line.startswith("end_hub:"):
                        zone = self.parse_zone(line, "end")
                        if self.end:
                            self.error(line_nb, "Multiple end_hub defined")
                        self.end = zone.name
                        self.zones[zone.name] = zone

                    elif line.startswith("hub:"):
                        zone = self.parse_zone(line, "normal")
                        if zone.name in self.zones:
                            self.error(line_nb, "Duplicate zone name")
                        self.zones[zone.name] = zone

                    elif line.startswith("connection:"):
                        a, b = self.parse_connection(line)
                        self.connections.append((a, b))

                    else:
                        self.error(line_nb, "Unknown line format")

        except Exception as e:
            print(f"Error: {e}")
            exit(1)

        self.build_graph()
        self.init_drones()

    def parse_nb_drones(self, line: str) -> int:
        """
        Extract the number of drones from a line.

        Args:
            line (str): Line containing nb_drones.

        Returns:
            int: Number of drones.

        Raises:
            ValueError: If the format is invalid.
        """
        try:
            return int(line.split(":")[1].strip())
        except Exception:
            raise ValueError("Invalid nb_drones format")

    def parse_zone(self, line: str, zone_kind: str) -> Zone:
        """
        Parse a zone definition line.

        Args:
            line (str): Line describing a zone.
            zone_kind (str): Type of zone declaration (start, end, normal).

        Returns:
            Zone: Parsed zone object.

        Raises:
            ValueError: If the format is invalid.
        """
        parts = line.split()

        if len(parts) < 4:
            raise ValueError("Invalid zone format")

        name = parts[1]
        x = int(parts[2])
        y = int(parts[3])

        metadata = self.parse_metadata(parts[4:])

        zone_type = metadata.get("zone", "normal")
        max_drones = int(metadata.get("max_drones", 1))
        color = metadata.get("color")

        if zone_kind in ("start", "end"):
            max_drones = float("inf")

        return Zone(name, x, y, zone_type, max_drones, color)

    def parse_connection(self, line: str) -> Tuple[str, str]:
        """
        Parse a connection line between two zones.

        Args:
            line (str): Line describing a connection.

        Returns:
            Tuple[str, str]: Pair of zone names.

        Raises:
            ValueError: If the format is invalid.
        """
        try:
            content = line.split()[1]
            a, b = content.split("-")
            return a, b
        except Exception:
            raise ValueError("Invalid connection format")

    def parse_metadata(self, parts: List[str]) -> Dict[str, str]:
        """
        Parse metadata enclosed in brackets.

        Args:
            parts (List[str]): Tokens containing metadata.

        Returns:
            Dict[str, str]: Parsed key-value metadata.

        Raises:
            ValueError: If metadata format is invalid.
        """
        if not parts:
            return {}

        raw = " ".join(parts)

        if not (raw.startswith("[") and raw.endswith("]")):
            raise ValueError("Invalid metadata format")

        content = raw[1:-1]
        metadata = {}

        for pair in content.split():
            if "=" not in pair:
                raise ValueError("Invalid metadata key=value")
            key, value = pair.split("=")
            metadata[key] = value

        return metadata

    def build_graph(self) -> None:
        """
        Build the graph by linking zones based on parsed connections.

        Raises:
            ValueError: If a connection references an unknown zone.
        """
        for a, b in self.connections:
            if a not in self.zones or b not in self.zones:
                raise ValueError(f"Connection uses unknown zone: {a}-{b}")

            self.zones[a].connections.append(self.zones[b])
            self.zones[b].connections.append(self.zones[a])

    def init_drones(self) -> None:
        if not self.start:
            raise ValueError("Start zone not defined")

        start_zone = self.zones[self.start]

        self.drones = [
            Drone(start_zone.x, start_zone.y)
            for _ in range(self.nb_drones)
        ]

    def error(self, line_nb: int, msg: str) -> None:
        """
        Raise a formatted parsing error.

        Args:
            line_nb (int): Line number where the error occurred.
            msg (str): Error description.

        Raises:
            ValueError: Always raised with formatted message.
        """
        raise ValueError(f"Line {line_nb}: {msg}")

    def grid_fill(self) -> None:
        for zone in self.zones.values():
            x0 = (zone.x - self.min_x) * self.CELL_W
            y0 = (zone.y - self.min_y) * self.CELL_H

            for i in range(self.CELL_W):
                self.grid[y0][x0 + i] = "#"
                self.grid[y0 + self.CELL_H - 1][x0 + i] = "#"

            for j in range(self.CELL_H):
                self.grid[y0 + j][x0] = "#"
                self.grid[y0 + j][x0 + self.CELL_W - 1] = "#"

    def render(self) -> None:
        display: list[list[str]] = [row[:] for row in self.grid]

        for drone in self.drones:
            cx, cy = self.to_screen(drone.x, drone.y)
            display[cy][cx] = "D"

        for row in display:
            for cell in row:
                print(cell, end="")
            print()

    def to_screen(self, x: int, y: int) -> tuple[int, int]:
        gx = x - self.min_x
        gy = y - self.min_y

        x0 = gx * self.CELL_W
        y0 = gy * self.CELL_H

        cx = x0 + self.CELL_W // 2
        cy = y0 + self.CELL_H // 2

        return cx, cy

    def simulate_positions(self) -> Generator[list[tuple[int, int]], None, None]:
        """
        Generator that yields all drone positions each turn,
        until every drone has reached the end zone.
        """
        if not self.path:
            raise ValueError("Path not computed.")

        path: list[Zone] = self.path
        n: int = self.nb_drones

        # Each drone starts at index 0 (start zone)
        positions: list[int] = [0] * n
        cooldowns: list[int] = [0] * n

        # Stagger departure: drone i waits i turns before starting
        departure_delay: list[int] = [i for i in range(n)]

        start_zone: Zone = path[0]
        for drone in self.drones:
            drone.x = start_zone.x
            drone.y = start_zone.y

        # Yield initial snapshot
        yield [(drone.x, drone.y) for drone in self.drones]

        while any(pos < len(path) - 1 for pos in positions):
            for i, drone in enumerate(self.drones):

                # Drone hasn't departed yet
                if departure_delay[i] > 0:
                    departure_delay[i] -= 1
                    continue

                # Drone already at end
                if positions[i] >= len(path) - 1:
                    continue

                # Drone is waiting due to movement cost
                if cooldowns[i] > 0:
                    cooldowns[i] -= 1
                    continue

                # Move to next zone
                next_index: int = positions[i] + 1
                next_zone: Zone = path[next_index]

                drone.x = next_zone.x
                drone.y = next_zone.y
                positions[i] = next_index

                cost: float = self.pathfinder.movement_cost(next_zone)
                cooldowns[i] = int(cost) - 1 if cost > 1 else 0

            yield [(drone.x, drone.y) for drone in self.drones]

    def simulate(self) -> None:
        for turn, drone_positions in enumerate(self.simulate_positions()):
            print(f"\n=== Turn {turn} ===")
            for i, (x, y) in enumerate(drone_positions):
                print(f"  Drone {i}: ({x}, {y})")
            self.render()
            time.sleep(1)
