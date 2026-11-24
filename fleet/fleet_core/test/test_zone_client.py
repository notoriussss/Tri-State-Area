"""Unit tests for Perry zone heartbeat, lease client, and free delivery slot selection."""

import math

import pytest
from fleet_core.constants import (
    CUBE_SIZE_M,
    SLOT_MIN_SEPARATION_M,
    ZONE_GREEN_LINE_X_M,
    ZONE_MAX_X_M,
    CubeColor,
    RobotName,
)
from fleet_core.types import CubeObservation
from fleet_core.zone_client import (
    ZoneClient,
    ZoneHeartbeat,
    is_call_expired,
    select_free_delivery_slot,
)


def test_should_expire_heartbeat_when_no_occupant_heard_for_2s() -> None:
    hb = ZoneHeartbeat(timeout_s=2.0)

    initial = hb.is_alive(now_s=1.0)
    hb.beat(now_s=1.0)
    fresh = hb.is_alive(now_s=2.9)
    stale = hb.is_alive(now_s=3.1)

    assert initial is False
    assert fresh is True
    assert stale is False


def test_should_block_new_request_when_gate_closed_but_finish_held_lease() -> None:
    hb = ZoneHeartbeat(timeout_s=2.0)
    hb.beat(now_s=5.0)
    client = ZoneClient(RobotName.PHINEAS.value, renew_period_s=20.0)
    client.update(want=True, gate_closed=True)

    req_when_closed = client.should_request(now_s=5.0)
    ready_when_closed = client.is_ready_to_enter(hb, now_s=5.0)

    client.update(occupant=RobotName.PHINEAS.value)
    ready_when_holding = client.is_ready_to_enter(hb, now_s=5.0)
    granted_when_holding = client.is_granted(hb, now_s=5.0)

    assert req_when_closed is False
    assert ready_when_closed is False
    assert ready_when_holding is True
    assert granted_when_holding is True


def test_should_renew_lease_and_retry_release_on_timeout() -> None:
    client = ZoneClient(RobotName.FERB.value, renew_period_s=20.0)
    client.update(want=True)

    first_req = client.should_request(now_s=10.0)
    client.note_requested(now_s=10.0)
    client.update(occupant=RobotName.FERB.value)
    before_renew = client.should_request(now_s=25.0)
    at_renew = client.should_request(now_s=30.0)

    client.request_release(CubeColor.YELLOW)
    must_release = client.should_release()
    delivered, color = client.pop_release()
    client.restore_release(delivered=delivered, color=color)
    must_retry_release = client.should_release()

    assert first_req is True
    assert before_renew is False
    assert at_renew is True
    assert must_release is True
    assert delivered is True
    assert color == CubeColor.YELLOW.value
    assert must_retry_release is True
    assert is_call_expired(10.0, 13.5, timeout_s=3.0) is True


def test_should_pick_free_delivery_slot_with_15mm_edge_clearance() -> None:
    occupied_center = CubeObservation(1, CubeColor.BLUE, 1.205, 0.355, 0.95)
    outside_cube = CubeObservation(2, CubeColor.YELLOW, 0.60, 0.355, 0.95)

    slot = select_free_delivery_slot((occupied_center, outside_cube))

    assert slot is not None
    assert ZONE_GREEN_LINE_X_M < slot[0] <= ZONE_MAX_X_M
    center_dist = math.hypot(slot[0] - occupied_center.x_m, slot[1] - occupied_center.y_m)
    assert center_dist >= CUBE_SIZE_M + SLOT_MIN_SEPARATION_M


def test_should_return_none_when_all_delivery_slots_are_occupied() -> None:
    full_cubes = (
        CubeObservation(1, CubeColor.BLUE, 1.205, 0.355, 0.95),
        CubeObservation(2, CubeColor.BLUE, 1.205, 0.310, 0.95),
        CubeObservation(3, CubeColor.BLUE, 1.205, 0.400, 0.95),
        CubeObservation(4, CubeColor.YELLOW, 1.245, 0.355, 0.95),
        CubeObservation(5, CubeColor.YELLOW, 1.245, 0.310, 0.95),
        CubeObservation(6, CubeColor.YELLOW, 1.245, 0.400, 0.95),
    )

    slot = select_free_delivery_slot(full_cubes)

    assert slot is None
    assert pytest.approx(ZONE_MAX_X_M) == 1.28
