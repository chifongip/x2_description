"""Check the calibrated Odin frame chain independently of capture artifacts."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


URDF = Path(__file__).resolve().parents[1] / 'urdf' / 'x2_ultra.urdf'


def joint_transform(joint):
    origin = joint.find('origin')
    translation = np.fromstring(origin.get('xyz'), sep=' ')
    roll, pitch, yaw = np.fromstring(origin.get('rpy'), sep=' ')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform[:3, 3] = translation
    return transform


def test_internal_geometry_matches_factory_calibration():
    root = ET.parse(URDF).getroot()
    il = joint_transform(root.find("joint[@name='odin_imu_to_lidar']"))
    expected_il = np.eye(4)
    expected_il[:3, 3] = [-0.02663, 0.03447, 0.02174]
    np.testing.assert_allclose(il, expected_il, atol=1e-12)

    # Device O1-P070100425: Tcl_0 maps LiDAR coordinates into camera.
    # Its decimal-rounded rotation must be projected onto SO(3) for URDF.
    cl = np.array([
        [-0.01267, -0.99991, -0.00533, 0.04060],
        [0.00056, 0.00533, -0.99999, -0.00472],
        [0.99992, -0.01267, 0.00049, -0.00794],
        [0.0, 0.0, 0.0, 1.0],
    ])
    u, _, vt = np.linalg.svd(cl[:3, :3])
    cl[:3, :3] = u @ vt
    lc = joint_transform(
        root.find("joint[@name='odin_lidar_to_camera_optical']")
    )
    np.testing.assert_allclose(lc @ cl, np.eye(4), atol=1e-12)


def test_torso_to_imu_matches_result_005():
    root = ET.parse(URDF).getroot()
    actual = joint_transform(root.find("joint[@name='torso_to_odin_imu']"))
    # Exported result_005 mount; keep this test independent of ignored datasets.
    expected = np.array(
        [
            [
                0.9928343990094166,
                -0.027558231824999967,
                -0.11627725487940602,
                0.07372875643220372,
            ],
            [
                0.029345006587433088,
                0.9994756941423114,
                0.013682375785229219,
                -0.03967478014480583,
            ],
            [
                0.1158392279497507,
                -0.016996490150153875,
                0.9931225466127438,
                0.5191515226525708,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_allclose(actual, expected, atol=1e-12)


def test_composed_torso_to_camera_pose():
    root = ET.parse(URDF).getroot()
    actual = np.eye(4)
    for name in (
        'torso_to_odin_imu',
        'odin_imu_to_lidar',
        'odin_lidar_to_camera_optical',
    ):
        actual = actual @ joint_transform(root.find(f"joint[@name='{name}']"))
    # Independent composition of the calibrated head camera pose and URDF FK.
    expected = np.array(
        [
            [
                0.015597976490395515,
                0.11668353106425511,
                0.9930466538424982,
                0.05161403546711954,
            ],
            [
                -0.9998260024416564,
                -0.008341917994204447,
                0.016684641012456584,
                0.034979182252264734,
            ],
            [
                0.010230736578646168,
                -0.9931341127876713,
                0.11653311137399237,
                0.5328936511470128,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_allclose(actual, expected, atol=1e-12)


def test_fixed_tree_has_unique_parentage_and_no_native_driver_frames():
    root = ET.parse(URDF).getroot()
    links = [link.get('name') for link in root.findall('link')]
    joints = root.findall('joint')
    children = [joint.find('child').get('link') for joint in joints]
    assert len(links) == len(set(links))
    assert len(children) == len(set(children))
    assert not {'imu', 'lidar', 'camera_0'}.intersection(links)
    expected = {
        'odin_imu': 'torso_link',
        'odin_lidar': 'odin_imu',
        'odin_camera_optical': 'odin_lidar',
    }
    for child, parent in expected.items():
        joint = next(j for j in joints if j.find('child').get('link') == child)
        assert joint.get('type') == 'fixed'
        assert joint.find('parent').get('link') == parent
        link = root.find(f"link[@name='{child}']")
        assert link.find('collision') is None
        assert link.find('inertial') is None
