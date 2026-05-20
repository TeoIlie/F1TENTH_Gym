"""Raceline and centerline representation with cubic spline interpolation."""

from __future__ import annotations

import pathlib
from typing import Optional

import numpy as np

from ..rendering import EnvRenderer
from .cubic_spline import CubicSpline2D


class Raceline:
    """A racing or centerline path defined by waypoints and a cubic spline.

    Attributes:
        n: Number of waypoints.
        ss: Arc length along the line.
        xs: X-coordinates of the waypoints.
        ys: Y-coordinates of the waypoints.
        yaws: Yaw angles at the waypoints.
        ks: Curvature at the waypoints.
        vxs: Velocity along the line.
        axs: Acceleration along the line.
        w_lefts: Left track width at each waypoint.
        w_rights: Right track width at each waypoint.
        length: Total length of the raceline (meters).
        spline: Cubic spline fitted through the waypoints.
    """

    def __init__(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        velxs: np.ndarray,
        ss: Optional[np.ndarray] = None,
        psis: Optional[np.ndarray] = None,
        kappas: Optional[np.ndarray] = None,
        accxs: Optional[np.ndarray] = None,
        spline: Optional[CubicSpline2D] = None,
        w_lefts: Optional[np.ndarray] = None,
        w_rights: Optional[np.ndarray] = None,
    ):
        """Initialize a Raceline.

        Args:
            xs: X-coordinates of the waypoints.
            ys: Y-coordinates of the waypoints.
            velxs: Velocity at each waypoint.
            ss: Arc length values; computed from ``xs``/``ys`` if not provided.
            psis: Yaw angles at the waypoints.
            kappas: Curvature at the waypoints.
            accxs: Acceleration at the waypoints.
            spline: Pre-built cubic spline; fitted from ``xs``/``ys`` if not provided.
            w_lefts: Left track width at each waypoint (centerline only).
            w_rights: Right track width at each waypoint (centerline only).
        """
        assert xs.shape == ys.shape == velxs.shape, "inconsistent shapes for x, y, vel"

        self.n = xs.shape[0]
        self.ss = ss
        self.xs = xs
        self.ys = ys
        self.yaws = psis
        self.ks = kappas
        self.vxs = velxs
        self.axs = accxs
        self.w_lefts = w_lefts
        self.w_rights = w_rights

        # approximate track length by linear-interpolation of x,y waypoints
        # note: we could use 'ss' but sometimes it is normalized to [0,1], so we recompute it here
        self.length = float(np.sum(np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)))

        # compute spline through waypoints if not provided
        self.spline = spline or CubicSpline2D(x=xs, y=ys)

        self.waypoint_render = None

    @staticmethod
    def from_centerline_file(
        filepath: pathlib.Path,
        delimiter: Optional[str] = ",",
        fixed_speed: Optional[float] = 1.0,
        track_scale: Optional[float] = 1.0,
    ):
        """Load a centerline from a CSV file.

        Expected format: ``[x, y, w_right, w_left]`` per row.
        Validates that the centerline is symmetric within 10% tolerance.

        Args:
            filepath: Path to the centerline CSV file.
            delimiter: Field delimiter (default ``","``).
            fixed_speed: Constant speed assigned to all waypoints (default 1.0).
            track_scale: Scale factor applied to coordinates (default 1.0).

        Returns:
            :class:`Raceline` instance representing the centerline.
        """
        assert filepath.exists(), f"input filepath does not exist ({filepath})"
        waypoints = np.loadtxt(filepath, delimiter=delimiter)
        assert waypoints.shape[1] == 4, "expected waypoints as [x, y, w_right, w_left]"

        # fit cubic spline to waypoints
        xx, yy = waypoints[:, 0], waypoints[:, 1]
        w_right_raw, w_left_raw = waypoints[:, 2], waypoints[:, 3]

        # scale waypoints
        xx, yy = xx * track_scale, yy * track_scale
        w_right_raw, w_left_raw = w_right_raw * track_scale, w_left_raw * track_scale

        # Validate centerline symmetry (within 10% tolerance)
        # Ensures the centerline is actually centered between track boundaries
        for i in range(len(w_right_raw)):
            max_width = max(w_right_raw[i], w_left_raw[i])
            if max_width > 0:  # Avoid division by zero
                diff_ratio = abs(w_right_raw[i] - w_left_raw[i]) / max_width
                if diff_ratio > 0.1:
                    raise ValueError(
                        f"Centerline validation failed at waypoint {i}: "
                        f"w_right={w_right_raw[i]:.3f}m, w_left={w_left_raw[i]:.3f}m "
                        f"(difference {diff_ratio * 100:.1f}% exceeds 10% tolerance). "
                        f"Centerline must be centered with symmetric track boundaries. "
                        f"File: {filepath}"
                    )

        # close loop
        xx = np.append(xx, xx[0])
        yy = np.append(yy, yy[0])
        w_right_raw = np.append(w_right_raw, w_right_raw[0])
        w_left_raw = np.append(w_left_raw, w_left_raw[0])

        spline = CubicSpline2D(x=xx, y=yy)
        ds = 0.1

        ss, xs, ys, yaws, ks, w_rights, w_lefts = [], [], [], [], [], [], []

        for i_s in np.arange(0, spline.s[-1], ds):
            x, y = spline.calc_position(i_s)
            yaw = spline.calc_yaw(i_s)
            k = spline.calc_curvature(i_s)

            # Find closest waypoint for width interpolation
            closest_idx = np.argmin(np.hypot(xx - x, yy - y))
            w_right = w_right_raw[closest_idx]
            w_left = w_left_raw[closest_idx]

            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
            ks.append(k)
            ss.append(i_s)
            w_rights.append(w_right)
            w_lefts.append(w_left)

        return Raceline(
            ss=np.array(ss).astype(np.float32),
            xs=np.array(xs).astype(np.float32),
            ys=np.array(ys).astype(np.float32),
            psis=np.array(yaws).astype(np.float32),
            kappas=np.array(ks).astype(np.float32),
            velxs=np.ones_like(ss).astype(np.float32) * fixed_speed,  # constant speed
            accxs=np.zeros_like(ss).astype(np.float32),  # constant acceleration
            spline=spline,
            w_lefts=np.array(w_lefts).astype(np.float32),
            w_rights=np.array(w_rights).astype(np.float32),
        )

    @staticmethod
    def from_raceline_file(
        filepath: pathlib.Path, delimiter: str = ";", track_scale: Optional[float] = 1.0
    ) -> Raceline:
        """Load an optimised raceline from a CSV file.

        Expected format: ``[s, x, y, psi, k, vx, ax]`` per row.

        Args:
            filepath: Path to the raceline CSV file.
            delimiter: Field delimiter (default ``";"``).
            track_scale: Scale factor applied to coordinates (default 1.0).

        Returns:
            :class:`Raceline` instance representing the optimised racing line.
        """
        assert filepath.exists(), f"input filepath does not exist ({filepath})"
        waypoints = np.loadtxt(filepath, delimiter=delimiter).astype(np.float32)

        if track_scale != 1.0:
            # scale x-y waypoints and recalculate s, psi, and k
            waypoints[:, 1] *= track_scale
            waypoints[:, 2] *= track_scale
            spline = CubicSpline2D(x=waypoints[:, 1], y=waypoints[:, 2])
            ss, yaws, ks = [], [], []
            for x, y in zip(waypoints[:, 1], waypoints[:, 2]):
                i_s, _ = spline.calc_arclength(x, y)
                yaw = spline.calc_yaw(i_s)
                k = spline.calc_curvature(i_s)
                yaws.append(yaw)
                ks.append(k)
                ss.append(i_s)
            waypoints[:, 0] = ss
            waypoints[:, 3] = yaws
            waypoints[:, 4] = ks

        assert waypoints.shape[1] == 7, "expected waypoints as [s, x, y, psi, k, vx, ax]"
        return Raceline(
            ss=waypoints[:, 0],
            xs=waypoints[:, 1],
            ys=waypoints[:, 2],
            psis=waypoints[:, 3],
            kappas=waypoints[:, 4],
            velxs=waypoints[:, 5],
            accxs=waypoints[:, 6],
        )

    def reversed(self) -> "Raceline":
        """Create a reversed copy of this raceline for reverse-direction driving.

        Works for both centerline (with ``w_lefts``/``w_rights``) and raceline
        (where those fields are ``None``). Track widths are swapped so left/right
        remain correct relative to the new driving direction.

        Returns:
            New :class:`Raceline` with reversed direction.
        """
        # Reverse coordinate arrays
        xs_rev = self.xs[::-1].copy()
        ys_rev = self.ys[::-1].copy()

        # Compute reversed arc lengths: re-parameterize from new starting point
        # Original: ss = [0, s1, s2, ..., L]
        # Reversed: ss_rev = [0, L-s_{n-1}, L-s_{n-2}, ..., L]
        ss_rev = None
        if self.ss is not None:
            total_length = self.ss[-1]
            ss_rev = (total_length - self.ss[::-1]).copy()

        # Create new spline from reversed coordinates
        spline_rev = CubicSpline2D(x=xs_rev, y=ys_rev)

        # Flip yaw by pi, wrap to [-pi, pi]
        yaws_rev = None
        if self.yaws is not None:
            yaws_rev = (self.yaws[::-1] + np.pi).copy()
            yaws_rev = np.arctan2(np.sin(yaws_rev), np.cos(yaws_rev))

        # Negate curvatures (left turns become right turns)
        ks_rev = -self.ks[::-1].copy() if self.ks is not None else None

        # Swap track widths (left boundary becomes right) - handles None for raceline files
        w_lefts_rev = self.w_rights[::-1].copy() if self.w_rights is not None else None
        w_rights_rev = self.w_lefts[::-1].copy() if self.w_lefts is not None else None

        # Reverse velocities/accelerations (keep magnitudes)
        # vxs is required, so provide default if None (should not happen in practice)
        vxs_rev = self.vxs[::-1].copy() if self.vxs is not None else np.ones(self.n, dtype=np.float32)
        axs_rev = self.axs[::-1].copy() if self.axs is not None else None

        return Raceline(
            ss=ss_rev,
            xs=xs_rev,
            ys=ys_rev,
            velxs=vxs_rev,
            psis=yaws_rev,
            kappas=ks_rev,
            accxs=axs_rev,
            spline=spline_rev,
            w_lefts=w_lefts_rev,
            w_rights=w_rights_rev,
        )

    def render_waypoints(self, e: EnvRenderer, color: tuple[int, int, int] = (0, 128, 0)) -> None:
        """Render the raceline waypoints as a closed line.

        Args:
            e: Environment renderer object.
            color: RGB color tuple for the line (default green ``(0, 128, 0)``).
        """
        points = np.stack([self.xs, self.ys], axis=1)
        if self.waypoint_render is None:
            self.waypoint_render = e.render_closed_lines(points, color=color, size=1)
        else:
            # PyQt renderer supports updateItems, Pygame may not
            if hasattr(self.waypoint_render, "updateItems"):
                self.waypoint_render.updateItems(points)
            else:
                # For Pygame renderer, re-render
                self.waypoint_render = e.render_closed_lines(points, color=color, size=1)
