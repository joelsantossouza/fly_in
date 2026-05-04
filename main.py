from Map import Map
import sys


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python3 main.py <map_file>")
        exit(1)

    map_file = sys.argv[1]

    try:
        fly_map = Map(map_file)
        fly_map.simulate()
    except Exception as e:
        print(f"Error: {e}")
        exit(1)


if __name__ == "__main__":
    main()
