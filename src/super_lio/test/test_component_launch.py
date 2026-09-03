import os
import signal
import subprocess
import sys
import time
import zlib
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from xml.etree import ElementTree

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


def component_ros_domain(use_sim_time, *, pid=None, worker=None, nonce=None):
    process_id = os.getpid() if pid is None else pid
    worker_id = os.environ.get("PYTEST_XDIST_WORKER", "main") if worker is None else worker
    unique_nonce = time.monotonic_ns() if nonce is None else nonce
    seed = f"{process_id}:{worker_id}:{int(use_sim_time)}:{unique_nonce}".encode()
    return zlib.crc32(seed) % 233


def cleanup_launch_process(process, *, graceful_timeout=5.0, kill_timeout=5.0):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=graceful_timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=kill_timeout)


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


def test_component_runtime_dependencies_are_declared():
    """Missing runtime launch/test dependencies would break an installed component test."""
    package_xml = Path(__file__).parents[1] / "package.xml"
    root = ElementTree.parse(package_xml).getroot()
    dependencies = {
        (element.tag, element.text)
        for element in root
        if element.tag in {"exec_depend", "test_depend"}
    }
    assert {
        ("exec_depend", "ament_index_python"),
        ("exec_depend", "launch"),
        ("exec_depend", "launch_ros"),
        ("test_depend", "ros2launch"),
        ("test_depend", "rosgraph_msgs"),
    } <= dependencies


def test_component_cleanup_terminates_the_entire_launch_process_group():
    """A timed-out launch parent must not leave a child that can satisfy another case."""
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "import subprocess, time; subprocess.Popen(['sleep', '60']); time.sleep(60)",
        ],
        start_new_session=True,
    )
    cleanup_launch_process(process, graceful_timeout=0.2, kill_timeout=0.2)
    assert process.poll() is not None
    with pytest.raises(ProcessLookupError):
        os.killpg(process.pid, 0)


def test_component_domain_is_unique_per_case_and_valid():
    """A fixed domain lets a stale true node answer a later false case."""
    true_domain = component_ros_domain(True, pid=1000, worker="gw0", nonce=1)
    false_domain = component_ros_domain(False, pid=1000, worker="gw0", nonce=2)
    assert 0 <= true_domain <= 232
    assert 0 <= false_domain <= 232
    assert true_domain != false_domain


@pytest.mark.parametrize("use_sim_time", [True, False])
def test_super_lio_component_exposes_requested_time_domain(use_sim_time, monkeypatch):
    """A component launch must preserve either explicit time-domain choice."""
    monkeypatch.setenv("ROS_DOMAIN_ID", str(component_ros_domain(use_sim_time)))
    process = None
    probe = None
    try:
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
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            cleanup_launch_process(process)
