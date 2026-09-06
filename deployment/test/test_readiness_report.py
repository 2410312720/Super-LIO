import os
from pathlib import Path
import subprocess


DEPLOYMENT = Path(__file__).parents[1]
RUNNER = DEPLOYMENT / "run.sh"


def test_readiness_report_dispatch_precedes_privileged_runner_setup():
    source = RUNNER.read_text()
    dispatch = source.index('if [[ "${1:-}" == "readiness-report" ]]')
    preflight = source.index('readonly PREFLIGHT_CHECKER=')
    privilege = source.index("sudo -v")

    assert dispatch < preflight < privilege
    prefix = source[:preflight]
    for forbidden in (
        "sudo ", "systemctl ", "ros2 topic pub", "VehicleCommand",
        "ros2 service call", "ros2 action send_goal",
    ):
        assert forbidden not in prefix


def test_readiness_report_forwards_arguments_without_preflight_or_sudo(tmp_path):
    workspace = tmp_path / "workspace"
    fake_bin = tmp_path / "bin"
    setup = workspace / "install" / "setup.bash"
    setup.parent.mkdir(parents=True)
    fake_bin.mkdir()
    capture = tmp_path / "ros2-arguments"
    setup.write_text(f'export PATH="{fake_bin}:$PATH"\n')
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        "#!/bin/sh\nprintf '%s\\n' \"$@\" >\"$READINESS_CAPTURE\"\n"
    )
    ros2.chmod(0o755)
    sudo = fake_bin / "sudo"
    sudo.write_text("#!/bin/sh\nexit 97\n")
    sudo.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        FLIGHT_WORKSPACE=str(workspace),
        READINESS_CAPTURE=str(capture),
        ROS_DOMAIN_ID="66",
    )

    completed = subprocess.run(
        [str(RUNNER), "readiness-report", "--timeout", "1.5", "--output", "/tmp/x"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0, completed.stderr
    assert capture.read_text().splitlines() == [
        "run", "aceflight_flight_bringup", "readiness-report",
        "--timeout", "1.5", "--output", "/tmp/x",
    ]
