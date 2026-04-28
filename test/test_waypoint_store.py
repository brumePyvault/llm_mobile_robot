import json

import pytest

from llm_mobile_robot.waypoint_store import WaypointStore


@pytest.fixture
def waypoint_json(tmp_path):
    path = tmp_path / 'waypoints.json'
    path.write_text(
        json.dumps(
            {
                'entrance': {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
                'office': {'x': 2.5, 'y': 1.0, 'yaw_deg': 90.0},
            }
        ),
        encoding='utf-8',
    )
    return path


def test_load_waypoints_from_json_fixture(waypoint_json):
    store = WaypointStore(waypoint_json)

    loaded = store.load()

    assert loaded['entrance']['x'] == 0.0
    assert loaded['office']['yaw_deg'] == 90.0


def test_save_waypoints_to_json(tmp_path):
    waypoint_path = tmp_path / 'saved_waypoints.json'
    store = WaypointStore(waypoint_path)

    store.set_waypoint('printer', {'x': 1.5, 'y': 1.0, 'yaw_deg': 45.0})
    store.save()

    payload = json.loads(waypoint_path.read_text(encoding='utf-8'))
    assert payload['printer']['x'] == 1.5
    assert payload['printer']['yaw_deg'] == 45.0
