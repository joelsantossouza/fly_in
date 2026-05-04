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
        zone_type (str): Type of the zone (normal, blocked, restricted,
            priority).
        max_drones (int): Maximum number of drones allowed simultaneously.
        color (str | None): Optional color for visualization.
        connections (List[Zone]): Adjacent zones (neighbors in the graph).
    """
    name: str
    x: int
    y: int
    zone_type: str = "normal"
    max_drones: int | float = 1
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

    def movement_cost(self, zone: "Zone") -> float:
        if zone.zone_type == "blocked":
            return float("inf")
        if zone.zone_type == "restricted":
            return 2.0
        if zone.zone_type == "priority":
            return 1.0
        return 1.0

    def heuristic(self, a: "Zone", b: "Zone") -> int:
        return abs(a.x - b.x) + abs(a.y - b.y)

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

                    priority_bonus: float = (
                        -0.1 if neighbor.zone_type == "priority" else 0.0
                    )

                    f_score[neighbor] = (
                        tentative_g
                        + float(self.heuristic(neighbor, goal))
                        + priority_bonus
                    )

                    counter += 1
                    heapq.heappush(
                        open_set, (f_score[neighbor], counter, neighbor))

        return []

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

    def path_cost(self, path: list["Zone"]) -> float:
        cost = 0.0
        for i in range(len(path) - 1):
            cost += self.movement_cost(path[i + 1])
        return cost

    def find_path(self) -> list[list["Zone"]]:
        assert self.map.start is not None
        assert self.map.end is not None

        start_zone: "Zone" = self.map.zones[self.map.start]
        end_zone: "Zone" = self.map.zones[self.map.end]

        P1 = self.a_star(start_zone, end_zone)
        if not P1:
            return []

        candidates = []

        for i in range(len(P1) - 1):
            spur_node = P1[i]
            root_path = P1[:i + 1]

            removed_edges = []

            for path in [P1]:
                if len(path) > i and path[:i + 1] == root_path:
                    a = path[i]
                    b = path[i + 1]
                    if b in a.connections:
                        a.connections.remove(b)
                        removed_edges.append((a, b))
                    if a in b.connections:
                        b.connections.remove(a)
                        removed_edges.append((b, a))

            spur_path = self.a_star(spur_node, end_zone)

            for a, b in removed_edges:
                a.connections.append(b)

            if spur_path:
                total_path = root_path[:-1] + spur_path
                candidates.append(total_path)

            if len(candidates) >= 1:
                break

        if not candidates:
            return [P1]

        P2 = min(candidates, key=lambda p: self.path_cost(p))

        if P2 == P1:
            return [P1]

        return [P1, P2]


VALID_ZONE_TYPES = {"normal", "blocked", "restricted", "priority"}


class Map:
    """
    Manages the Fly-in map, including parsing input files and building the
    graph.

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
        self.paths: list[list[Zone]] = self.pathfinder.find_path()

    def register_zone(self, zone: "Zone", line_nb: int) -> None:
        if zone.name in self.zones:
            self.error(line_nb, f"Duplicate zone name: '{zone.name}'")
        if any(z.x == zone.x and z.y == zone.y for z in self.zones.values()):
            self.error(
                line_nb, f"Duplicate zone position: ({zone.x}, {zone.y})")
        self.zones[zone.name] = zone

    def parse(self, filepath: str) -> None:
        nb_drones_defined = False

        try:
            with open(filepath, "r") as file:
                for line_nb, line in enumerate(file, 1):
                    line = line.strip()

                    if not line or line.startswith("#"):
                        continue

                    if line.startswith("nb_drones:"):
                        if nb_drones_defined:
                            self.error(
                                line_nb, "nb_drones defined more than once")
                        self.nb_drones = self.parse_nb_drones(line)
                        nb_drones_defined = True

                    elif not nb_drones_defined:
                        self.error(
                            line_nb, "nb_drones must be the first definition")

                    elif line.startswith("start_hub:"):
                        zone = self.parse_zone(line, "start")
                        if self.start:
                            self.error(line_nb, "Multiple start_hub defined")
                        self.register_zone(zone, line_nb)
                        self.start = zone.name

                    elif line.startswith("end_hub:"):
                        zone = self.parse_zone(line, "end")
                        if self.end:
                            self.error(line_nb, "Multiple end_hub defined")
                        self.register_zone(zone, line_nb)
                        self.end = zone.name

                    elif line.startswith("hub:"):
                        zone = self.parse_zone(line, "normal")
                        self.register_zone(zone, line_nb)

                    elif line.startswith("connection:"):
                        a, b = self.parse_connection(line, line_nb)
                        if ((a, b) in self.connections
                                or (b, a) in self.connections):
                            self.error(
                                line_nb, f"Duplicate connection: {a}-{b}"
                            )
                        self.connections.append((a, b))

                    else:
                        self.error(line_nb, "Unknown line format")

        except Exception as e:
            print(f"Error: {e}")
            exit(1)

        if not self.start:
            print("Error: Missing start_hub definition")
            exit(1)
        if not self.end:
            print("Error: Missing end_hub definition")
            exit(1)

        self.build_graph()
        self.init_drones()

    def parse_nb_drones(self, line: str) -> int:
        try:
            value = int(line.split(":")[1].strip())
        except Exception:
            raise ValueError("Invalid nb_drones format")
        if value <= 0:
            raise ValueError("nb_drones must be a positive integer")
        return value

    def parse_zone(self, line: str, zone_kind: str) -> Zone:
        parts = line.split()

        if len(parts) < 4:
            raise ValueError("Invalid zone format")

        name = parts[1]
        if "-" in name or " " in name:
            raise ValueError(
                f"Zone name '{name}' must not contain dashes or spaces")

        try:
            x = int(parts[2])
            y = int(parts[3])
        except ValueError:
            raise ValueError(f"Zone '{name}' has invalid coordinates")

        metadata = self.parse_metadata(parts[4:])

        zone_type = metadata.get("zone", "normal")
        if zone_type not in VALID_ZONE_TYPES:
            raise ValueError(
                f"Invalid zone type: '{zone_type}'. "
                f"Must be one of: {', '.join(VALID_ZONE_TYPES)}"
            )

        max_drones_raw = metadata.get("max_drones", "1")
        if not max_drones_raw.isdigit() or int(max_drones_raw) <= 0:
            raise ValueError(f"max_drones must be a positive integer, got '{
                             max_drones_raw}'")
        max_drones: int | float = int(max_drones_raw)

        color = metadata.get("color")

        if zone_kind in ("start", "end"):
            max_drones = float("inf")

        return Zone(name, x, y, zone_type, max_drones, color)

    def parse_connection(self, line: str, line_nb: int) -> Tuple[str, str]:
        try:
            parts = line.split()
            content = parts[1]
            a, b = content.split("-")
        except Exception:
            raise ValueError("Invalid connection format")

        if a not in self.zones:
            raise ValueError(f"Connection references undefined zone: '{a}'")
        if b not in self.zones:
            raise ValueError(f"Connection references undefined zone: '{b}'")

        metadata = self.parse_metadata(parts[2:])
        if "max_link_capacity" in metadata:
            cap = metadata["max_link_capacity"]
            if not cap.isdigit() or int(cap) <= 0:
                raise ValueError(
                    "max_link_capacity must be a positive integer, "
                    f"got '{cap}'"
                )

        return a, b

    def parse_metadata(self, parts: List[str]) -> Dict[str, str]:
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
            key, value = pair.split("=", 1)
            metadata[key] = value

        return metadata

    def build_graph(self) -> None:
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

    def colorize(self, text: str, color: Optional[str]) -> str:
        if color is None:
            return text

        ansi_colors = {
            "black": "\033[30m",
            "red": "\033[31m",
            "green": "\033[32m",
            "yellow": "\033[33m",
            "blue": "\033[34m",
            "magenta": "\033[35m",
            "cyan": "\033[36m",
            "white": "\033[37m",
            "gray": "\033[90m",
            "orange": "\033[38;5;208m",
            "purple": "\033[38;5;93m",
            "brown": "\033[38;5;94m",
            "maroon": "\033[38;5;52m",
            "gold": "\033[38;5;220m",
            "darkred": "\033[38;5;88m",
            "violet": "\033[38;5;177m",
            "crimson": "\033[38;5;160m",
            "pink": "\033[38;5;213m",
        }

        reset = "\033[0m"

        color = color.lower()

        if color == "rainbow":
            rainbow_codes = [
                "\033[31m",
                "\033[33m",
                "\033[32m",
                "\033[36m",
                "\033[34m",
                "\033[35m",
            ]
            return "".join(
                f"{rainbow_codes[i % len(rainbow_codes)]}{c}"
                for i, c in enumerate(text)
            ) + reset

        code = ansi_colors.get(color, "\033[37m")

        return f"{code}{text}{reset}"

    def grid_fill(self) -> None:
        for zone in self.zones.values():
            x0 = (zone.x - self.min_x) * self.CELL_W
            y0 = (zone.y - self.min_y) * self.CELL_H

            border = self.colorize("#", zone.color)

            for i in range(self.CELL_W):
                self.grid[y0][x0 + i] = border
                self.grid[y0 + self.CELL_H - 1][x0 + i] = border

            for j in range(self.CELL_H):
                self.grid[y0 + j][x0] = border
                self.grid[y0 + j][x0 + self.CELL_W - 1] = border

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

    def simulate_positions(self
                           ) -> Generator[list[tuple[int, int]], None, None]:
        """
        Generator that yields all drone positions each turn,
        enforcing zone capacity limits and multi-turn zone traversal,
        with even/odd drones using different shortest paths.
        """
        if not self.paths:
            raise ValueError("Path not computed.")

        n: int = self.nb_drones

        positions: list[int] = [0] * n

        departure_delay: list[int] = [i for i in range(n)]

        zone_travel_remaining: dict[tuple[int, int], int] = {}

        start_zone: Zone = self.paths[0][0]
        for drone in self.drones:
            drone.x = start_zone.x
            drone.y = start_zone.y

        yield [(drone.x, drone.y) for drone in self.drones]

        def drone_not_finished(idx: int) -> bool:
            path = self.paths[0] if idx % 2 == 0 else self.paths[-1]
            return positions[idx] < len(path) - 1

        while any(drone_not_finished(i) for i in range(n)):

            zone_occupancy: dict[tuple[int, int], int] = {}
            for drone in self.drones:
                pos = (drone.x, drone.y)
                zone_occupancy[pos] = zone_occupancy.get(pos, 0) + 1

            drone_order = sorted(
                range(n), key=lambda i: positions[i], reverse=True)

            for i in drone_order:
                drone = self.drones[i]
                path = self.paths[0] if i % 2 == 0 else self.paths[-1]

                if departure_delay[i] > 0:
                    departure_delay[i] -= 1
                    continue

                if positions[i] >= len(path) - 1:
                    continue

                next_index = positions[i] + 1
                next_zone = path[next_index]
                next_pos = (next_zone.x, next_zone.y)

                if zone_travel_remaining.get(next_pos, 0) > 0:
                    continue

                current_count = zone_occupancy.get(next_pos, 0)
                if current_count >= next_zone.max_drones:
                    continue

                old_pos = (drone.x, drone.y)
                zone_occupancy[old_pos] -= 1

                drone.x = next_zone.x
                drone.y = next_zone.y
                positions[i] = next_index

                zone_occupancy[next_pos] = current_count + 1

                travel_time = int(self.pathfinder.movement_cost(next_zone))
                if travel_time > 1:
                    zone_travel_remaining[next_pos] = max(
                        zone_travel_remaining.get(next_pos, 0),
                        travel_time - 1
                    )

            for z in list(zone_travel_remaining.keys()):
                if zone_travel_remaining[z] > 0:
                    zone_travel_remaining[z] -= 1

            yield [(drone.x, drone.y) for drone in self.drones]

    def simulate(self) -> None:
        """
        Outputs simulation logs in the required format:
        D<ID>-<zone> or D<ID>-<connection>
        Only drones that move in a turn are printed.
        Drones that reach the end zone disappear.
        """

        assert self.start is not None

        previous_positions = [(drone.x, drone.y) for drone in self.drones]

        delivered = [False] * self.nb_drones

        for turn, drone_positions in enumerate(self.simulate_positions()):

            print(f"=== Turn {turn} ===")
            movements = []

            for i, (x, y) in enumerate(drone_positions):

                if delivered[i]:
                    continue

                old_x, old_y = previous_positions[i]

                if (x, y) != (old_x, old_y):

                    next_zone = None
                    for z in self.zones.values():
                        if z.x == x and z.y == y:
                            next_zone = z
                            break

                    if next_zone is None:
                        continue

                    if next_zone.name == self.end:
                        movements.append(f"D{i+1}-{next_zone.name}")
                        delivered[i] = True
                        continue

                    if next_zone.zone_type == "restricted":
                        connection_name = f"{
                            self.zones[self.start].name}-{next_zone.name}"
                        movements.append(f"D{i+1}-{connection_name}")

                    else:
                        movements.append(f"D{i+1}-{next_zone.name}")

            if movements:
                print(" ".join(movements))

            previous_positions = drone_positions

            self.render()
            time.sleep(1)

            if all(delivered):
                break
