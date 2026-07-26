"""Headless end-to-end test for delivery at Gazebo delivery area B."""

import math
import os
from pathlib import Path
import re
import subprocess
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from delivery_robot_interfaces.msg import RobotState
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


ROBOT_MODEL_NAME = 'robot_a'
MISSION_TIMEOUT_ENV = 'GAZEBO_MISSION_TEST_TIMEOUT'
DEFAULT_MISSION_TIMEOUT = 480.0
MAX_MISSION_TIMEOUT = 520.0
GROUND_TRUTH_QUERY_TIMEOUT = 30.0
GROUND_TRUTH_COMMAND_TIMEOUT = 8.0
POSITION_EPSILON = 0.02

_NUMBER = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
_COORDINATE_SEPARATOR = r'(?:\s*\|\s*|\s+)'
_XYZ_PATTERN = re.compile(
    rf'\[\s*({_NUMBER}){_COORDINATE_SEPARATOR}'
    rf'({_NUMBER}){_COORDINATE_SEPARATOR}({_NUMBER})\s*\]'
)


def generate_test_description():
    """Launch the complete scenario with both graphical clients disabled."""
    ros_domain = re.sub(
        r'[^A-Za-z0-9_]',
        '_',
        os.environ.get('ROS_DOMAIN_ID', 'unset'),
    )
    gz_partition = f'delivery_mission_{os.getpid()}_{ros_domain}'
    os.environ['GZ_PARTITION'] = gz_partition

    package_share = Path(
        get_package_share_directory('delivery_robot_description')
    )
    simulation_launch = package_share / 'launch' / 'simulation.launch.py'

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(simulation_launch)),
        launch_arguments={
            'gui': 'false',
            'rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


def _parse_pose(element):
    """Return an SDF pose as x, y, yaw, defaulting omitted poses to zero."""
    if element is None or not element.text:
        return 0.0, 0.0, 0.0
    values = [float(value) for value in element.text.split()]
    values.extend([0.0] * (6 - len(values)))
    return values[0], values[1], values[5]


def _compose_pose(parent, child):
    """Compose two planar poses."""
    parent_x, parent_y, parent_yaw = parent
    child_x, child_y, child_yaw = child
    cosine = math.cos(parent_yaw)
    sine = math.sin(parent_yaw)
    return (
        parent_x + cosine * child_x - sine * child_y,
        parent_y + sine * child_x + cosine * child_y,
        parent_yaw + child_yaw,
    )


def _load_delivery_area():
    """Derive delivery area B's rectangular footprint from the installed SDF."""
    package_share = Path(
        get_package_share_directory('delivery_robot_description')
    )
    world_path = package_share / 'worlds' / 'world.sdf'
    root = ET.parse(world_path).getroot()
    model = root.find("./world/model[@name='delivery_area_b']")
    if model is None:
        raise AssertionError(
            f'delivery_area_b is missing from Gazebo world {world_path}'
        )

    marker = model.find(
        "./link[@name='delivery_area']/visual[@name='marker']"
    )
    if marker is None:
        raise AssertionError(
            f'delivery_area_b marker is missing from Gazebo world {world_path}'
        )
    size_element = marker.find('./geometry/box/size')
    if size_element is None or not size_element.text:
        raise AssertionError(
            f'delivery_area_b marker has no box size in {world_path}'
        )
    size = [float(value) for value in size_element.text.split()]
    if len(size) != 3 or size[0] <= 0.0 or size[1] <= 0.0:
        raise AssertionError(
            f'delivery_area_b marker has invalid box size in {world_path}'
        )

    link = model.find("./link[@name='delivery_area']")
    model_pose = _parse_pose(model.find('./pose'))
    link_pose = _parse_pose(link.find('./pose'))
    marker_pose = _parse_pose(marker.find('./pose'))
    center_pose = _compose_pose(
        _compose_pose(model_pose, link_pose),
        marker_pose,
    )
    return center_pose, size[0], size[1], world_path


def _mission_timeout():
    """Read and cap a positive finite mission timeout from the environment."""
    raw_value = os.environ.get(
        MISSION_TIMEOUT_ENV,
        str(DEFAULT_MISSION_TIMEOUT),
    )
    try:
        timeout = float(raw_value)
    except ValueError as error:
        raise AssertionError(
            f'{MISSION_TIMEOUT_ENV} must be a positive finite number'
        ) from error
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise AssertionError(
            f'{MISSION_TIMEOUT_ENV} must be a positive finite number'
        )
    return min(timeout, MAX_MISSION_TIMEOUT)


def _parse_ground_truth_pose(output):
    """Parse the XYZ row printed by Gazebo's model command."""
    match = _XYZ_PATTERN.search(output)
    if match is None:
        return None
    return tuple(float(value) for value in match.groups())


def _query_ground_truth_pose(remaining_time):
    """Query Gazebo's entity state instead of using ROS odometry or TF."""
    if remaining_time <= 0.0:
        return None, 'ground-truth query deadline expired'
    command = ['gz', 'model', '-m', ROBOT_MODEL_NAME, '--pose']
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            check=False,
            text=True,
            timeout=min(GROUND_TRUTH_COMMAND_TIMEOUT, remaining_time),
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        return None, f'{type(error).__name__}: {error}'

    output = '\n'.join((completed.stdout, completed.stderr))
    if completed.returncode != 0:
        return None, (
            f'command exited {completed.returncode}: {output.strip()}'
        )
    pose = _parse_ground_truth_pose(output)
    if pose is None:
        return None, f'could not parse model pose from: {output.strip()}'
    return pose, ''


def _point_inside_area(point, center_pose, width, depth):
    """Check a world-frame point against B's possibly rotated footprint."""
    point_x, point_y = point
    center_x, center_y, center_yaw = center_pose
    delta_x = point_x - center_x
    delta_y = point_y - center_y
    cosine = math.cos(center_yaw)
    sine = math.sin(center_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return (
        abs(local_x) <= width / 2.0 + POSITION_EPSILON
        and abs(local_y) <= depth / 2.0 + POSITION_EPSILON
    )


class GazeboPoseParserTest(unittest.TestCase):
    """Check the Gazebo CLI formats accepted by the ground-truth parser."""

    def test_xyz_formats(self):
        """Parse current space-separated and legacy pipe-separated rows."""
        cases = (
            (
                'Model: [8]\n'
                '- Pose [ XYZ (m) ] [ RPY (rad) ]:\n'
                '[1.250000 -7.800000 0.000000]\n'
                '[0.000000 0.000000 0.000000]\n',
                (1.25, -7.8, 0.0),
            ),
            (
                '- Pose [ XYZ (m) ] [ RPY (rad) ]:\n'
                '[1.250000 | -7.800000 | 0.000000]\n',
                (1.25, -7.8, 0.0),
            ),
        )
        for output, expected in cases:
            with self.subTest(output=output):
                self.assertEqual(expected, _parse_ground_truth_pose(output))


class GazeboDeliveryMissionTest(unittest.TestCase):
    """Verify mission success and the independent Gazebo world pose."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS once for the launch test process."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut ROS down even when the assertion body fails."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Create a reliable subscription to the mission state."""
        self.node = rclpy.create_node('gazebo_delivery_mission_test')
        self.latest_state = None
        self.state_count = 0
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_subscription = self.node.create_subscription(
            RobotState,
            '/delivery_robot/state',
            self._on_state,
            state_qos,
        )

    def tearDown(self):
        """Destroy test-owned ROS entities before launch teardown."""
        self.node.destroy_subscription(self.state_subscription)
        self.node.destroy_node()

    def _on_state(self, message):
        self.latest_state = message
        self.state_count += 1

    def _wait_for_arrival(self):
        deadline = time.monotonic() + _mission_timeout()
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.latest_state is None:
                continue
            if self.latest_state.delivery_state == RobotState.FAILED:
                self.fail(
                    'delivery mission reported FAILED at '
                    f'({self.latest_state.pose.x:.3f}, '
                    f'{self.latest_state.pose.y:.3f}) after '
                    f'{self.state_count} state messages'
                )
            if self.latest_state.delivery_state == RobotState.ARRIVED:
                return

        latest = 'none'
        if self.latest_state is not None:
            latest = (
                f'state={self.latest_state.delivery_state}, '
                f'pose=({self.latest_state.pose.x:.3f}, '
                f'{self.latest_state.pose.y:.3f})'
            )
        self.fail(
            f'mission did not reach ARRIVED within {_mission_timeout():.1f}s; '
            f'latest={latest}, messages={self.state_count}'
        )

    def _wait_for_ground_truth_pose(self):
        deadline = time.monotonic() + GROUND_TRUTH_QUERY_TIMEOUT
        last_error = 'no query attempted'
        while time.monotonic() < deadline:
            remaining_time = deadline - time.monotonic()
            pose, last_error = _query_ground_truth_pose(remaining_time)
            if pose is not None:
                return pose
            sleep_time = min(0.5, max(0.0, deadline - time.monotonic()))
            if sleep_time > 0.0:
                time.sleep(sleep_time)
        self.fail(
            'Gazebo ground-truth pose was unavailable after ARRIVED: '
            f'{last_error}'
        )

    def test_robot_arrives_inside_delivery_area_b(self):
        """Require ARRIVED and independently verify world position inside B."""
        center_pose, width, depth, world_path = _load_delivery_area()
        self._wait_for_arrival()
        ground_truth = self._wait_for_ground_truth_pose()

        self.assertTrue(
            _point_inside_area(
                (ground_truth[0], ground_truth[1]),
                center_pose,
                width,
                depth,
            ),
            (
                f'Gazebo ground-truth robot pose '
                f'({ground_truth[0]:.3f}, {ground_truth[1]:.3f}) is outside '
                f'delivery area B centered at '
                f'({center_pose[0]:.3f}, {center_pose[1]:.3f}) with '
                f'size ({width:.3f}, {depth:.3f}); source={world_path}'
            ),
        )
