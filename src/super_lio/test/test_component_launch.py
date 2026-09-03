import os
import signal
import subprocess
import time
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
import rclpy
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch import LaunchContext
from launch_ros.actions import Node
from launch.utilities import perform_substitutions
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "super_lio_component.launch.py"
os.environ.setdefault("ROS_LOG_DIR", "/tmp/aceflight_launch_test_logs")


def describe_component_launch():
    spec = spec_from_file_location("super_lio_component_launch", LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package_name: f"/tmp/{package_name}"
    return module.generate_launch_description()


def node_keys(nodes):
    context = LaunchContext()
    return {
        (
            node.node_package,
            node._Node__node_name
            if isinstance(node._Node__node_name, str)
            else perform_substitutions(context, node._Node__node_name),
        )
        for node in nodes
    }


def test_super_lio_component_owns_exactly_one_node():
    """Removing the component node or adding another owned resource is a bug."""
    description = describe_component_launch()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert node_keys(nodes) == {
        ("super_lio", "super_lio_node")
    }
    assert not [
        entity for entity in description.entities if type(entity) is ExecuteProcess
    ]


def test_super_lio_component_declares_explicit_parameters():
    """Removing either component interface argument would break independent bringup."""
    description = describe_component_launch()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {"config_file", "use_sim_time"}


@pytest.mark.parametrize("use_sim_time", [True, False])
def test_super_lio_component_exposes_requested_time_domain(use_sim_time, monkeypatch):
    """A component launch must preserve either explicit time-domain choice."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "41" if use_sim_time else "42")
    process = subprocess.Popen(
        [
            "ros2", "launch", "super_lio", "super_lio_component.launch.py",
            f"use_sim_time:={str(use_sim_time).lower()}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    probe = rclpy.create_node("super_lio_component_time_probe")
    clock = probe.create_publisher(Clock, "/clock", 10)
    client = probe.create_client(GetParameters, "/super_lio_node/get_parameters")
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.1):
            clock.publish(Clock())
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert client.service_is_ready(), "super_lio_node parameter service did not become ready"
        request = GetParameters.Request()
        request.names = ["use_sim_time"]
        future = client.call_async(request)
        while time.monotonic() < deadline and not future.done():
            clock.publish(Clock())
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert future.done(), "use_sim_time parameter request timed out"
        assert future.result().values[0].bool_value is use_sim_time
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
