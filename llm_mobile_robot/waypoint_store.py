import json
from pathlib import Path
from typing import Any


class WaypointStore:
    """JSON-backed waypoint storage utility."""

    def __init__(self, file_path: str | Path) -> None:
        self.file_path = Path(file_path)
        self.waypoints: dict[str, dict[str, float]] = {}

    def load(self) -> dict[str, dict[str, float]]:
        if not self.file_path.exists():
            self.waypoints = {}
            return self.waypoints

        with self.file_path.open('r', encoding='utf-8') as stream:
            raw: Any = json.load(stream)

        if not isinstance(raw, dict):
            raise ValueError('Waypoint JSON must be an object keyed by waypoint name.')

        parsed: dict[str, dict[str, float]] = {}
        for name, pose in raw.items():
            if not isinstance(pose, dict):
                raise ValueError(f"Waypoint '{name}' must be a JSON object.")
            parsed[name] = {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'yaw_deg': float(pose.get('yaw_deg', 0.0)),
            }

        self.waypoints = parsed
        return self.waypoints

    def save(self) -> None:
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        with self.file_path.open('w', encoding='utf-8') as stream:
            json.dump(self.waypoints, stream, indent=2, sort_keys=True)
            stream.write('\n')

    def set_waypoint(self, name: str, pose: dict[str, float]) -> None:
        self.waypoints[name] = {
            'x': float(pose['x']),
            'y': float(pose['y']),
            'yaw_deg': float(pose.get('yaw_deg', 0.0)),
        }