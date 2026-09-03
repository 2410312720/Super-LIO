from contextlib import contextmanager
import fcntl
import os
import select
import signal
import subprocess
import sys
import time
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


@contextmanager
def component_ros_domain(_use_sim_time, lock_directory=Path("/tmp")):
    """Lease a non-default DDS domain until the component test has cleaned up."""
    lock_directory = Path(lock_directory)
    for domain in range(1, 233):
        lock_handle = open(
            lock_directory / f"aceflight_component_ros_domain_{domain}.lock", "w"
        )
        try:
            fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock_handle.close()
            continue
        try:
            yield domain
        finally:
            fcntl.flock(lock_handle, fcntl.LOCK_UN)
            lock_handle.close()
        return
    raise RuntimeError("no non-default ROS domain is available for component test")


def process_group_exists(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def cleanup_launch_process(process, pgid, *, graceful_timeout=5.0, kill_timeout=5.0):
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=graceful_timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=kill_timeout)
    deadline = time.monotonic() + kill_timeout
    while process_group_exists(pgid) and time.monotonic() < deadline:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    if process_group_exists(pgid):
        raise TimeoutError(f"launch process group {pgid} did not exit")


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
    """A SIGTERM-resistant child must die even when its launch parent exits first."""
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            (
                "import subprocess, sys, time; "
                "subprocess.Popen([sys.executable, '-c', "
                "'import os, signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                "print(f\\\"READY {os.getpid()}\\\", flush=True); time.sleep(60)']); "
                "time.sleep(60)"
            ),
        ],
        start_new_session=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    pgid = process.pid
    child_pid = None
    try:
        readable, _, _ = select.select([process.stdout], [], [], 2.0)
        assert readable, "SIGTERM-resistant child did not report READY"
        ready = process.stdout.readline().strip()
        assert ready.startswith("READY ")
        child_pid = int(ready.split()[1])
        cleanup_launch_process(process, pgid, graceful_timeout=0.2, kill_timeout=1.0)
        assert process.poll() is not None
        with pytest.raises(ProcessLookupError):
            os.killpg(pgid, 0)
        with pytest.raises(ProcessLookupError):
            os.kill(child_pid, 0)
    finally:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=1.0)


def test_component_domain_is_unique_per_case_and_valid(tmp_path):
    """Independent worker processes lease distinct non-default DDS domains."""
    worker_code = """
import importlib.util
import sys
spec = importlib.util.spec_from_file_location('component_test', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
with module.component_ros_domain(True, lock_directory=sys.argv[2]) as domain:
    print(domain, flush=True)
    sys.stdin.readline()
"""
    workers = []
    try:
        for _ in range(2):
            workers.append(subprocess.Popen(
                [sys.executable, "-c", worker_code, str(Path(__file__)), str(tmp_path)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True,
            ))
        domains = [int(worker.stdout.readline().strip()) for worker in workers]
        assert all(1 <= domain <= 232 for domain in domains)
        assert domains[0] != domains[1]
    finally:
        for worker in workers:
            if worker.stdin is not None:
                try:
                    worker.stdin.write("\n")
                    worker.stdin.flush()
                except BrokenPipeError:
                    pass
            worker.wait(timeout=1.0)
    with component_ros_domain(True, lock_directory=tmp_path) as reused_domain:
        assert reused_domain in domains
    with component_ros_domain(True) as runtime_domain:
        assert 1 <= runtime_domain <= 232


@pytest.mark.parametrize("use_sim_time", [True, False])
def test_super_lio_component_exposes_requested_time_domain(use_sim_time, monkeypatch):
    """A component launch must preserve either explicit time-domain choice."""
    with component_ros_domain(use_sim_time) as domain:
        monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
        process = None
        process_group_id = None
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
            process_group_id = process.pid
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
                cleanup_launch_process(process, process_group_id)
