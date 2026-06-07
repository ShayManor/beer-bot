import pytest


@pytest.mark.parametrize(
    "module, cls",
    [
        ("autonomous_rover.nodes.camera.camera_node", "CameraNode"),
        ("autonomous_rover.nodes.pathfinder.pathfinder_node", "PathfinderNode"),
        ("autonomous_rover.nodes.localization.localization_node", "LocalizationNode"),
        ("autonomous_rover.nodes.e_comms.e_comms_node", "ECommsNode"),
        ("autonomous_rover.nodes.master.master_node", "MasterNode"),
    ],
)
def test_node_constructs(ros_ctx, module, cls):
    import importlib

    mod = importlib.import_module(module)
    node_cls = getattr(mod, cls)
    with ros_ctx():
        node = node_cls()
        node.destroy_node()
