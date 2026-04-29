from dataclasses import dataclass, field
from typing import Dict, List, Tuple


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

    def __init__(self, filepath: str) -> None:
        """Initialize an empty map."""
        self.zones: Dict[str, Zone] = {}
        self.connections: List[Tuple[str, str]] = []
        self.nb_drones: int = 0
        self.start: str | None = None
        self.end: str | None = None
        self.parse(filepath)

        zones_lst = list(self.zones.values())
        self.min_x = min(z.x for z in zones_lst)
        self.max_x = max(z.x for z in zones_lst)
        self.min_y = min(z.y for z in zones_lst)
        self.max_y = max(z.y for z in zones_lst)
        self.width = self.max_x - self.min_x + 1
        self.height = self.max_y - self.min_y + 1
        self.grid = [
            ["0" for _ in range(self.width)]
            for _ in range(self.height)
        ]
        self.grid_fill()

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
            x: int = zone.x - self.min_x
            y: int = zone.y - self.min_y
            self.grid[y][x] = '$'

    def render(self, drones: list) -> None:
        display: list[list[str]] = self.grid[:]

        for drone in drones:
            display[drone.y][drone.x] = '#'

        for row in display:
            for cell in row:
                print(cell, end="")
            print()
