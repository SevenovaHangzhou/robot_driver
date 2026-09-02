import sys

from launch.launch_description_sources import (
    get_launch_description_from_any_launch_file,
)


def main():
    description = get_launch_description_from_any_launch_file(sys.argv[1])
    if not description.entities:
        raise RuntimeError("launch description contains no entities")


if __name__ == "__main__":
    main()
