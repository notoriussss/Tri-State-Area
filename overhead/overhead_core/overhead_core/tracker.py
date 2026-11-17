"""Multi-object tracking with greedy nearest-neighbor data association for cubes."""

import math
from dataclasses import dataclass

from overhead_core.constants import (
    DEFAULT_GRACE_S,
    DEFAULT_MAX_JUMP_M,
)


@dataclass
class _Track:
    """Internal representation of an active cube track."""

    track_id: int
    color: str
    x: float
    y: float
    confidence: float
    initial_conf: float
    last_seen_s: float


class CubeTracker:
    """Tracks detected cubes across frames using greedy nearest-neighbor association."""

    def __init__(
        self,
        max_jump_m: float = DEFAULT_MAX_JUMP_M,
        grace_s: float = DEFAULT_GRACE_S,
    ) -> None:
        """Initializes the cube tracker.

        Args:
            max_jump_m: Maximum distance in meters between consecutive detections of the same cube.
            grace_s: Duration in seconds to retain an unmatched track before dropping it.
        """
        self._max_jump_m = max_jump_m
        self._grace_s = grace_s
        self._next_id = 1
        self._tracks: dict[int, _Track] = {}

    def _find_candidate_pairs(
        self,
        tracks: list[_Track],
        detections: list[tuple[str, float, float, float]],
    ) -> list[tuple[float, int, int]]:
        """Finds candidate match pairs within max_jump_m Euclidean distance."""
        candidate_pairs: list[tuple[float, int, int]] = []
        for track_idx, track in enumerate(tracks):
            for det_idx, det in enumerate(detections):
                dist = math.hypot(
                    track.x - det[1],
                    track.y - det[2],
                )
                if dist <= self._max_jump_m:
                    candidate_pairs.append(
                        (
                            dist,
                            track_idx,
                            det_idx,
                        )
                    )
        candidate_pairs.sort(key=lambda p: p[0])
        return candidate_pairs

    def _match_detections(
        self,
        tracks: list[_Track],
        detections: list[tuple[str, float, float, float]],
        now_s: float,
    ) -> set[int]:
        """Matches candidate pairs greedily and updates matched tracks."""
        candidate_pairs = self._find_candidate_pairs(
            tracks,
            detections,
        )
        matched_track_indices: set[int] = set()
        matched_det_indices: set[int] = set()

        for _dist, t_idx, d_idx in candidate_pairs:
            if t_idx in matched_track_indices or d_idx in matched_det_indices:
                continue
            matched_track_indices.add(t_idx)
            matched_det_indices.add(d_idx)

            track = tracks[t_idx]
            det = detections[d_idx]
            track.x = float(det[1])
            track.y = float(det[2])
            track.confidence = float(det[3])
            track.initial_conf = float(det[3])
            track.last_seen_s = now_s

        return matched_det_indices

    def _spawn_tracks(
        self,
        detections: list[tuple[str, float, float, float]],
        matched_indices: set[int],
        now_s: float,
    ) -> None:
        """Creates new tracks for unmatched detections."""
        for d_idx, det in enumerate(detections):
            if d_idx not in matched_indices:
                new_track = _Track(
                    track_id=self._next_id,
                    color=str(det[0]),
                    x=float(det[1]),
                    y=float(det[2]),
                    confidence=float(det[3]),
                    initial_conf=float(det[3]),
                    last_seen_s=now_s,
                )
                self._tracks[self._next_id] = new_track
                self._next_id += 1

    def _expire_tracks(self, now_s: float) -> None:
        """Drops tracks exceeding grace_s and decays confidence of unobserved tracks."""
        expired_track_ids: list[int] = []
        for track_id, track in self._tracks.items():
            dt_s = now_s - track.last_seen_s
            if dt_s > 0.0:
                if dt_s > self._grace_s:
                    expired_track_ids.append(track_id)
                else:
                    decay_ratio = max(
                        0.0,
                        1.0 - (dt_s / self._grace_s),
                    )
                    track.confidence = track.initial_conf * decay_ratio

        for track_id in expired_track_ids:
            del self._tracks[track_id]

    def _collect_active_tracks(self) -> list[tuple[int, str, float, float, float]]:
        """Returns a sorted list of active tracks."""
        active = [
            (
                t.track_id,
                t.color,
                t.x,
                t.y,
                t.confidence,
            )
            for t in self._tracks.values()
        ]
        active.sort(key=lambda item: item[0])
        return active

    def update(
        self,
        detections: list[tuple[str, float, float, float]],
        now_s: float,
    ) -> list[tuple[int, str, float, float, float]]:
        """Updates tracks with a new batch of detections.

        Matches detections to tracks greedily per color. Unmatched tracks survive
        for up to grace_s with decaying confidence. New cubes are assigned incrementing IDs.

        Args:
            detections: List of detections as (color, x_m, y_m, confidence).
            now_s: Current timestamp in seconds.

        Returns:
            List of active tracks as (track_id, color, x_m, y_m, confidence).
        """
        all_colors = {t.color for t in self._tracks.values()} | {d[0] for d in detections}

        for color in all_colors:
            tracks_of_color = [t for t in self._tracks.values() if t.color == color]
            dets_of_color = [d for d in detections if d[0] == color]
            matched_indices = self._match_detections(
                tracks_of_color,
                dets_of_color,
                now_s,
            )
            self._spawn_tracks(
                dets_of_color,
                matched_indices,
                now_s,
            )

        self._expire_tracks(now_s)
        return self._collect_active_tracks()
