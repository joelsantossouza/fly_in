from Map import Map

import os
from typing import List


def run_all_maps_recursive(root_dir: str) -> None:
    """
    Recursively find all .txt map files under root_dir
    and run Map(file).simulate() for each.
    """
    map_files: List[str] = []

    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(".txt"):
                full_path = os.path.join(dirpath, filename)
                map_files.append(full_path)

    map_files.sort()

    for file in map_files:
        print(f"\n=== Running map: {file} ===")
        try:
            map_obj: Map = Map(file)
            map_obj.simulate()
        except Exception:
            continue


if __name__ == "__main__":
    run_all_maps_recursive("maps/custom")
