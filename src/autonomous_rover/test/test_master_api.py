import pytest

pytest.importorskip("flask")


def _client(ros_ctx):
    from autonomous_rover.nodes.master.master_node import MasterNode

    node = MasterNode()
    return node, node.app.test_client()


def test_health(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        assert client.get("/health").get_json() == {"ok": True}
        node.destroy_node()


def test_index_page(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        r = client.get("/")
        assert r.status_code == 200
        assert "text/html" in r.content_type
        html = r.get_data(as_text=True)
        assert "ROVER" in html
        # teleop magnitudes are substituted into the page
        assert "__TELEOP_V__" not in html and "__TELEOP_W__" not in html
        node.destroy_node()


def test_teleop(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        assert client.post("/teleop", json={"v": "x"}).status_code == 400
        r = client.post("/teleop", json={"v": 0.3, "omega": -0.5})
        assert r.status_code == 200
        assert r.get_json() == {"v": 0.3, "omega": -0.5}
        node.destroy_node()


def test_state_validation_and_set(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        assert client.post("/state", json={"state": "nope"}).status_code == 400
        r = client.post("/state", json={"state": "active"})
        assert r.status_code == 200
        assert client.get("/status").get_json()["state"] == "active"
        node.destroy_node()


def test_goal_validation_and_publish(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        assert client.post("/goal", json={}).status_code == 400
        assert client.post("/goal", json={"x": "a", "y": 2}).status_code == 400
        r = client.post("/goal", json={"x": 1.5, "y": -2.0})
        assert r.status_code == 200
        body = r.get_json()
        assert body["accepted"] == {"x": 1.5, "y": -2.0, "frame_id": "map"}
        # posting a goal flips the robot active
        assert client.get("/status").get_json()["state"] == "active"
        node.destroy_node()


def test_status_and_map_shape(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        status = client.get("/status").get_json()
        for key in ("state", "pose", "heading", "path", "speed", "goal"):
            assert key in status
        cloud = client.get("/map").get_json()
        assert cloud == {"points": [], "count": 0, "truncated": False, "frame_id": None}
        node.destroy_node()


def test_logs_record_events(ros_ctx):
    with ros_ctx():
        node, client = _client(ros_ctx)
        client.post("/goal", json={"x": 0.0, "y": 0.0})
        logs = client.get("/logs").get_json()["logs"]
        assert any("goal" in entry["msg"] for entry in logs)
        node.destroy_node()


def test_debug_image_endpoint(ros_ctx):
    from sensor_msgs.msg import CompressedImage

    with ros_ctx():
        node, client = _client(ros_ctx)
        # No frame yet -> 503.
        assert client.get("/debug_image").status_code == 503
        # Inject a frame the way the subscription would.
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = b"\xff\xd8\xff\xd9"  # minimal JPEG-ish bytes
        node._on_debug_image(msg)
        r = client.get("/debug_image")
        assert r.status_code == 200
        assert r.content_type == "image/jpeg"
        assert r.get_data() == b"\xff\xd8\xff\xd9"
        node.destroy_node()


def test_camera_image_endpoint(ros_ctx):
    from sensor_msgs.msg import CompressedImage

    with ros_ctx():
        node, client = _client(ros_ctx)
        # No frame yet -> 503.
        assert client.get("/camera_image").status_code == 503
        # Inject a frame the way the subscription would.
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = b"\xff\xd8\xff\xd9"  # minimal JPEG-ish bytes
        node._on_camera_image(msg)
        r = client.get("/camera_image")
        assert r.status_code == 200
        assert r.content_type == "image/jpeg"
        assert r.get_data() == b"\xff\xd8\xff\xd9"
        node.destroy_node()
