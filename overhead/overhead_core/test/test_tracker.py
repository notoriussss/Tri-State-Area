"""Unit tests for multi-cube tracking and data association."""

import math

from overhead_core.tracker import CubeTracker


def test_should_maintain_stable_id_when_cube_moves_slightly():
    tracker = CubeTracker(
        max_jump_m=0.05,
        grace_s=2.0,
    )
    first_dets = [("blue", 0.50, 0.30, 0.9)]
    second_dets = [("blue", 0.52, 0.31, 0.9)]

    tracks_t0 = tracker.update(
        first_dets,
        now_s=0.0,
    )
    tracks_t1 = tracker.update(
        second_dets,
        now_s=0.1,
    )

    assert len(tracks_t0) == 1
    assert len(tracks_t1) == 1
    assert tracks_t0[0][0] == tracks_t1[0][0]
    assert math.isclose(
        tracks_t1[0][2],
        0.52,
    )


def test_should_survive_occlusion_when_under_grace_period():
    tracker = CubeTracker(
        max_jump_m=0.05,
        grace_s=2.0,
    )
    initial_dets = [("yellow", 0.40, 0.20, 1.0)]

    tracker.update(
        initial_dets,
        now_s=0.0,
    )
    tracks_after_1s = tracker.update(
        [],
        now_s=1.0,
    )

    assert len(tracks_after_1s) == 1
    assert tracks_after_1s[0][0] == 1
    assert math.isclose(
        tracks_after_1s[0][4],
        0.5,
        abs_tol=0.05,
    )


def test_should_drop_track_when_occlusion_exceeds_grace_period():
    tracker = CubeTracker(
        max_jump_m=0.05,
        grace_s=2.0,
    )
    initial_dets = [("red", 0.70, 0.50, 0.8)]

    tracker.update(
        initial_dets,
        now_s=0.0,
    )
    tracks_after_grace = tracker.update(
        [],
        now_s=2.5,
    )

    assert len(tracks_after_grace) == 0


def test_should_increment_ids_when_new_cubes_appear():
    tracker = CubeTracker(
        max_jump_m=0.05,
        grace_s=2.0,
    )
    detections = [
        ("blue", 0.20, 0.20, 0.9),
        ("yellow", 0.80, 0.50, 0.9),
    ]

    tracks = tracker.update(
        detections,
        now_s=0.0,
    )

    assigned_ids = [t[0] for t in tracks]
    assert assigned_ids == [1, 2]
