import pytest


@pytest.mark.parametrize(
    "module, cls",
    [
        ("beer_bot.nodes.camera.camera_node", "CameraNode"),
        ("beer_bot.nodes.pathfinder.pathfinder_node", "PathfinderNode"),
        ("beer_bot.nodes.localization.localization_node", "LocalizationNode"),
        ("beer_bot.nodes.e_comms.e_comms_node", "ECommsNode"),
        ("beer_bot.nodes.master.master_node", "MasterNode"),
    ],
)
def test_node_constructs(ros_ctx, module, cls):
    import importlib

    mod = importlib.import_module(module)
    node_cls = getattr(mod, cls)
    with ros_ctx():
        node = node_cls()
        node.destroy_node()
