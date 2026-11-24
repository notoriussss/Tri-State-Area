"""Unit tests for the pure stateful WorldPipeline."""

import cv2
import numpy as np
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    ObstacleKind,
    RobotName,
    VirtualSensorEvent,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
)
from overhead_core.world_frame import WorldFrameConfig
from overhead_core.world_pipeline import WorldPipeline
from overhead_core.world_safety import SafetyConfig


def _make_arena_scene(
    *,
    include_phineas: bool = True,
    include_ferb: bool = True,
    include_red_cube: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    frame = np.full(
        (600, 900, 3),
        230,
        dtype=np.uint8,
    )
    cv2.rectangle(
        frame,
        (40, 30),
        (860, 570),
        (0, 0, 0),
        20,
    )
    if include_phineas:
        phineas_pts = np.array(
            [
                [315, 300],
                [255, 235],
                [255, 365],
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(
            frame,
            [phineas_pts],
            (0, 0, 255),
        )
    if include_ferb:
        ferb_pts = np.array(
            [
                [615, 300],
                [555, 235],
                [555, 365],
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(
            frame,
            [ferb_pts],
            (0, 75, 255),
        )
    if include_red_cube:
        cv2.rectangle(
            frame,
            (375, 290),
            (395, 310),
            (0, 0, 255),
            -1,
        )

    corners = detect_arena_corners(frame)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    return frame, homography


def test_should_detect_cubes_and_robots_when_synthetic_arena_frame_processed() -> None:
    frame, homography = _make_arena_scene()
    pipeline = WorldPipeline(WorldFrameConfig())

    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )

    assert detection.is_all_seen is True
    assert set(detection.seen_robots.keys()) == {
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    }
    assert len(detection.robot_poses) == 2
    assert len(detection.tracked_cubes) == 1


def test_should_hold_robot_then_drop_when_robot_disappears_past_hold_timeout() -> None:
    both_frame, homography = _make_arena_scene()
    phineas_only_frame, _ = _make_arena_scene(include_ferb=False)
    pipeline = WorldPipeline(
        WorldFrameConfig(),
        hold_s=0.5,
    )

    initial = pipeline.detect_and_track(
        both_frame,
        homography=homography,
        now_s=1.0,
    )
    held = pipeline.detect_and_track(
        phineas_only_frame,
        homography=homography,
        now_s=1.25,
    )
    dropped = pipeline.detect_and_track(
        phineas_only_frame,
        homography=homography,
        now_s=1.6,
    )

    assert initial.is_all_seen is True
    assert held.is_all_seen is False
    assert {pose.name for pose in held.robot_poses} == {
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    }
    assert dropped.is_all_seen is False
    assert [pose.name for pose in dropped.robot_poses] == [RobotName.PHINEAS.value]


def test_should_evaluate_sensors_and_red_obstacles_when_safety_evaluated() -> None:
    frame, homography = _make_arena_scene()
    zone_rect = (0.0, 0.0, 0.5, 0.7)
    pipeline = WorldPipeline(
        WorldFrameConfig(),
        zone_rect=zone_rect,
        near_red_m=0.5,
    )
    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )

    evaluation = pipeline.evaluate_sensors_and_safety(
        detection,
        now_s=1.0,
        safety_config=SafetyConfig(zone_rect=zone_rect),
    )

    assert RobotName.PHINEAS.value in evaluation.vs_result["robots_in_zone"]
    phineas_events = evaluation.vs_result["per_robot"][RobotName.PHINEAS.value][2]
    assert VirtualSensorEvent.ENTERED_ZONE.value in phineas_events
    assert len(evaluation.red_obstacles) == 1
    assert evaluation.red_obstacles[0].kind == ObstacleKind.RED_CUBE
