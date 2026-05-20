"""Track loading, Frenet coordinate conversion, and rendering."""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import yaml
from PIL import Image
from PIL.Image import Transpose
from yamldataclassconfig.config import YamlDataClassConfig

from ..rendering import EnvRenderer
from . import Raceline
from .cubic_spline import CubicSpline2D
from .track_utils import find_track_dir


@dataclass
class TrackSpec(YamlDataClassConfig):
    """Track metadata loaded from a ROS-format YAML map file."""

    name: Optional[str]
    image: Optional[str]
    resolution: float
    origin: Tuple[float, float, float]
    negate: int
    occupied_thresh: float
    free_thresh: float


@dataclass
class Track:
    """Racing circuit with occupancy map, centerline, and raceline.

    Load via :meth:`from_track_name` or :meth:`from_track_path`. Supports
    Frenet coordinate conversion and reverse-direction driving.
    """

    spec: TrackSpec
    filepath: Optional[str]
    ext: Optional[str]
    occupancy_map: np.ndarray
    centerline: Raceline
    raceline: Raceline

    def __init__(
        self,
        spec: TrackSpec,
        occupancy_map: np.ndarray,
        filepath: Optional[str] = None,
        ext: Optional[str] = None,
        centerline: Optional[Raceline] = None,
        raceline: Optional[Raceline] = None,
    ):
        """Initialize a Track object.

        Args:
            spec: Track specification loaded from YAML.
            occupancy_map: Binary occupancy grid (0 = obstacle, 255 = free).
            filepath: Path to the track image (without extension).
            ext: File extension of the track image.
            centerline: Centerline raceline (geometric centre of track).
            raceline: Optimised racing line; defaults to centerline if not provided.
        """
        self.spec = spec
        self.filepath = filepath
        self.ext = ext
        self.occupancy_map = occupancy_map

        # Store regular versions (raceline defaults to centerline if not provided)
        self.centerline_regular = centerline
        self.raceline_regular = raceline if raceline is not None else centerline

        # Compute reversed versions once
        self.centerline_reversed = centerline.reversed() if centerline else None
        if self.raceline_regular is self.centerline_regular:
            # if raceline is a copy of centerline, the reversed should reference the same reversed object
            self.raceline_reversed = self.centerline_reversed
        else:
            # else, if raceline is different from centerline, it should be calculated from scratch
            self.raceline_reversed = self.raceline_regular.reversed() if self.raceline_regular else None

        # Active references (default to regular direction)
        self.centerline = self.centerline_regular
        self.raceline = self.raceline_regular

        # Render handle for lookahead curvature visualization
        self.lookahead_render = None

        # Arc length annotation rendering cache
        self.arc_annotation_points_render = None
        self.arc_annotation_text_renders = []

        # Debug: Store last projected point for visualization
        self.debug_projected_point = None
        self.debug_vehicle_point = None
        self.debug_render_projected = None
        self.debug_render_vehicle = None
        self.debug_render_line = None

    def set_direction(self, reversed: bool) -> None:
        """
        Set track direction by swapping active centerline/raceline references.

        Args:
            reversed: If True, use reversed versions. If False, use regular versions.
        """
        if reversed:
            self.centerline = self.centerline_reversed
            self.raceline = self.raceline_reversed
        else:
            self.centerline = self.centerline_regular
            self.raceline = self.raceline_regular

    @staticmethod
    def load_spec(track: str, filespec: str) -> TrackSpec:
        """Load track specification from a YAML file.

        Args:
            track: Track name identifier.
            filespec: Path to the YAML metadata file.

        Returns:
            Populated :class:`TrackSpec` instance.
        """
        with open(filespec, "r") as yaml_stream:
            map_metadata = yaml.safe_load(yaml_stream)
            track_spec = TrackSpec(name=track, **map_metadata)
        return track_spec

    @staticmethod
    def from_track_name(track: str, track_scale: float = 1.0) -> Track:
        """Load a track by name from the maps directory or user cache.

        Args:
            track: Track name (must exist in the local ``maps/`` submodule or
                ``~/.gymkhana/maps/``).
            track_scale: Scale factor applied to coordinates (default 1.0).

        Returns:
            Loaded :class:`Track` instance.

        Raises:
            FileNotFoundError: If the track files cannot be found.
        """
        try:
            track_dir = find_track_dir(track)
            track_spec = Track.load_spec(track=track, filespec=str(track_dir / f"{track_dir.stem}_map.yaml"))
            track_spec.resolution = track_spec.resolution * track_scale
            track_spec.origin = (
                track_spec.origin[0] * track_scale,
                track_spec.origin[1] * track_scale,
                track_spec.origin[2],
            )

            # load occupancy grid
            map_filename = pathlib.Path(track_spec.image)
            image = Image.open(track_dir / str(map_filename)).transpose(Transpose.FLIP_TOP_BOTTOM)

            occupancy_map = np.array(image)
            # Handle boolean (1-bit) images by scaling to 0-255 range
            if occupancy_map.dtype == bool:
                occupancy_map = occupancy_map.astype(np.float32) * 255.0
            else:
                occupancy_map = occupancy_map.astype(np.float32)
            occupancy_map[occupancy_map <= 128] = 0.0
            occupancy_map[occupancy_map > 128] = 255.0

            # if exists, load centerline
            if (track_dir / f"{track}_centerline.csv").exists():
                centerline = Raceline.from_centerline_file(
                    track_dir / f"{track}_centerline.csv",
                    track_scale=track_scale,
                )
            else:
                centerline = None

            # if exists, load raceline (otherwise None, __init__ will use centerline as fallback)
            if (track_dir / f"{track}_raceline.csv").exists():
                raceline = Raceline.from_raceline_file(
                    track_dir / f"{track}_raceline.csv",
                    track_scale=track_scale,
                )
            else:
                raceline = None

            return Track(
                spec=track_spec,
                filepath=str((track_dir / map_filename.stem).absolute()),
                ext=map_filename.suffix,
                occupancy_map=occupancy_map,
                centerline=centerline,
                raceline=raceline,
            )
        except Exception as ex:
            print(ex)
            raise FileNotFoundError(f"It could not load track {track}") from ex

    @staticmethod
    def from_track_path(path: pathlib.Path, track_scale: float = 1.0) -> Track:
        """Load a track from a YAML file path.

        Args:
            path: Path to the track YAML metadata file.
            track_scale: Scale factor applied to coordinates (default 1.0).

        Returns:
            Loaded :class:`Track` instance.

        Raises:
            FileNotFoundError: If the track files cannot be found.
        """
        try:
            if type(path) is str:
                path = pathlib.Path(path)

            track_spec = Track.load_spec(track=path.stem, filespec=path)
            track_spec.resolution = track_spec.resolution * track_scale
            track_spec.origin = (
                track_spec.origin[0] * track_scale,
                track_spec.origin[1] * track_scale,
                track_spec.origin[2],
            )

            # load occupancy grid
            # Image path is from path + image name from track_spec
            image_path = path.parent / track_spec.image
            image = Image.open(image_path).transpose(Transpose.FLIP_TOP_BOTTOM)
            occupancy_map = np.array(image)
            # Handle boolean (1-bit) images by scaling to 0-255 range
            if occupancy_map.dtype == bool:
                occupancy_map = occupancy_map.astype(np.float32) * 255.0
            else:
                occupancy_map = occupancy_map.astype(np.float32)
            occupancy_map[occupancy_map <= 128] = 0.0
            occupancy_map[occupancy_map > 128] = 255.0

            # if exists, load centerline
            if (path / f"{path.stem}_centerline.csv").exists():
                centerline = Raceline.from_centerline_file(path / f"{path.stem}_centerline.csv")
            else:
                centerline = None

            # if exists, load raceline (otherwise None, __init__ will use centerline as fallback)
            if (path / f"{path.stem}_raceline.csv").exists():
                raceline = Raceline.from_raceline_file(path / f"{path.stem}_raceline.csv")
            else:
                raceline = None

            return Track(
                spec=track_spec,
                filepath=str(path.absolute()),
                ext=image_path.suffix,
                occupancy_map=occupancy_map,
                centerline=centerline,
                raceline=raceline,
            )
        except Exception as ex:
            print(ex)
            raise FileNotFoundError(f"It could not load track {path}") from ex

    @staticmethod
    def from_refline(x: np.ndarray, y: np.ndarray, velx: np.ndarray):
        """Create a track from a reference line defined by waypoints and velocities.

        Args:
            x: X-coordinates of the waypoints.
            y: Y-coordinates of the waypoints.
            velx: Velocity at each waypoint.

        Returns:
            :class:`Track` instance with a synthetic occupancy map.
        """
        ds = 0.1
        resolution = 0.05
        margin_perc = 0.1

        spline = CubicSpline2D(x=x, y=y)
        ss, xs, ys, yaws, ks, vxs = [], [], [], [], [], []
        for i_s in np.arange(0, spline.s[-1], ds):
            xi, yi = spline.calc_position(i_s)
            yaw = spline.calc_yaw(i_s)
            k = spline.calc_curvature(i_s)

            # find closest waypoint
            closest = np.argmin(np.hypot(x - xi, y - yi))
            v = velx[closest]

            xs.append(xi)
            ys.append(yi)
            yaws.append(yaw)
            ks.append(k)
            ss.append(i_s)
            vxs.append(v)

        refline = Raceline(
            ss=np.array(ss).astype(np.float32),
            xs=np.array(xs).astype(np.float32),
            ys=np.array(ys).astype(np.float32),
            psis=np.array(yaws).astype(np.float32),
            kappas=np.array(ks).astype(np.float32),
            velxs=np.array(vxs).astype(np.float32),
            accxs=np.zeros_like(ss).astype(np.float32),
            spline=spline,
        )

        min_x, max_x = np.min(xs), np.max(xs)
        min_y, max_y = np.min(ys), np.max(ys)
        x_range = max_x - min_x
        y_range = max_y - min_y
        occupancy_map = 255.0 * np.ones(
            (
                int((1 + 2 * margin_perc) * x_range / resolution),
                int((1 + 2 * margin_perc) * y_range / resolution),
            ),
            dtype=np.float32,
        )
        # origin is the bottom left corner
        origin = (min_x - margin_perc * x_range, min_y - margin_perc * y_range, 0.0)

        track_spec = TrackSpec(
            name=None,
            image=None,
            resolution=resolution,
            origin=origin,
            negate=False,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )

        return Track(
            spec=track_spec,
            filepath=None,
            ext=None,
            occupancy_map=occupancy_map,
            raceline=refline,
            centerline=refline,
        )

    def frenet_to_cartesian(self, s, ey, ephi, use_raceline=False):
        """Convert Frenet coordinates to Cartesian.

        Args:
            s: Arc length distance along the centerline/raceline (meters).
            ey: Lateral deviation from the reference line (meters).
            ephi: Heading deviation (vehicle yaw minus track yaw, radians).
            use_raceline: If True, convert relative to raceline; else centerline.

        Returns:
            Tuple ``(x, y, psi)`` — Cartesian position and yaw angle.
        """
        line = self.raceline if use_raceline else self.centerline
        x, y = line.spline.calc_position(s)
        psi = line.spline.calc_yaw(s)

        # Adjust x,y by shifting along the normal vector
        x -= ey * np.sin(psi)
        y += ey * np.cos(psi)

        # Adjust psi by adding the heading deviation
        psi += ephi

        return x, y, psi

    def cartesian_to_frenet(self, x, y, phi, use_raceline=False, s_guess=0, precise=False, debug=False):
        """Convert Cartesian coordinates to Frenet coordinates.

        Args:
            x: Vehicle x-position (from ``std_state["x"]``).
            y: Vehicle y-position (from ``std_state["y"]``).
            phi: Vehicle yaw angle (from ``std_state["yaw"]``).
            use_raceline: If True, project onto raceline; else centerline.
            s_guess: Initial arc length guess for the optimisation-based method.
            precise: If True, use optimisation-based arc length search (requires
                a good ``s_guess``; may fail if the guess is poor).
            debug: If True, print validation info comparing precise vs. global search.

        Returns:
            Tuple ``(s, ey, ephi)`` — arc length, lateral deviation (meters),
            and heading deviation (radians).
        """
        # precise=True uses an optimisation-based search which can fail without a
        # good initial guess. The default global search (inaccurate) is more robust.
        line = self.raceline if use_raceline else self.centerline

        # Store vehicle position for visualization
        self.debug_vehicle_point = np.array([x, y], dtype=np.float32)

        # Optimization-based arclength calculation

        if precise:
            # This is more precise, but causes errors - the search sometimes does not find the correct spot on the map
            s, ey = line.spline.calc_arclength(x, y, s_guess)
        else:
            # This is slightly less accurate, but the global search doesn't cause errors
            s, ey = line.spline.calc_arclength_inaccurate(x, y)  # slightly more inaccurate, but much faster

        # Wrap around
        s = s % line.spline.s[-1]

        # Use the normal to calculate the signed lateral deviation
        yaw = line.spline.calc_yaw(s)
        normal = np.asarray([-np.sin(yaw), np.cos(yaw)])
        x_eval, y_eval = line.spline.calc_position(s)
        dx = x - x_eval
        dy = y - y_eval
        distance_sign = np.sign(np.dot([dx, dy], normal))
        ey = ey * distance_sign

        # Store projected point for visualization
        self.debug_projected_point = np.array([x_eval, y_eval], dtype=np.float32)

        # Validation: Check if optimization likely failed
        actual_distance = np.hypot(dx, dy)

        if debug:
            track_length = line.spline.s[-1]

            # Always compute robust method for comparison when using precise mode
            if precise:
                # Validate optimization against global search
                s_robust, ey_robust = line.spline.calc_arclength_inaccurate(x, y)
                s_robust = s_robust % track_length

                s_error = abs(s - s_robust)
                ey_error = abs(ey - ey_robust)

                print("[DEBUG Frenet] Method: PRECISE (optimization)")
                print(f"[DEBUG Frenet] Vehicle: ({x:.2f}, {y:.2f}) -> Projected: ({x_eval:.2f}, {y_eval:.2f})")
                print(f"[DEBUG Frenet] s_guess={s_guess:.2f}m → s_optimized={s:.2f}m")
                print(f"[DEBUG Frenet] Validation: s_robust={s_robust:.2f}m (global search)")
                print(f"[DEBUG Frenet] ey_precise={ey:.3f}m, ey_robust={ey_robust:.3f}m")
                print(
                    f"[DEBUG Frenet] Discrepancy: Δs={s_error:.2f}m, Δey={ey_error:.3f}m, dist={actual_distance:.3f}m"
                )

            # Common warnings (applies to both methods)
            if actual_distance > 10.0:
                print(f"[WARNING] Vehicle far from track! Distance={actual_distance:.2f}m (typical track width ~10m)")

        phi = phi - yaw
        return s, ey, np.arctan2(np.sin(phi), np.cos(phi))

    def render_centerline(self, e: EnvRenderer) -> None:
        """Render the track centerline in green.

        Args:
            e: Environment renderer object.
        """
        GREEN = (0, 255, 0)
        if self.centerline is not None:
            self.centerline.render_waypoints(e, color=GREEN)

    def render_raceline(self, e: EnvRenderer) -> None:
        """Render the track raceline in red.

        Args:
            e: Environment renderer object.
        """
        RED = (255, 0, 0)
        if self.raceline is not None:
            self.raceline.render_waypoints(e, color=RED)

    def render_both_lines(self, e: EnvRenderer) -> None:
        """Render both the centerline (green) and raceline (red).

        Args:
            e: Environment renderer object.
        """
        self.render_centerline(e)
        self.render_raceline(e)

    def render_arc_length_annotations(
        self,
        e: EnvRenderer,
        interval: float = 5.0,
        point_color: tuple[int, int, int] = (255, 165, 0),  # Orange
        text_color: tuple[int, int, int] = (255, 165, 0),  # Orange
        point_size: int = 6,
        font_size: int = 10,
    ) -> None:
        """Render arc length annotations along the centerline at regular intervals.

        Useful for identifying positions along the track in Frenet coordinates.

        Args:
            e: Environment renderer object.
            interval: Distance between annotations in meters (default 5.0).
            point_color: RGB color for annotation points (default orange).
            text_color: RGB color for text labels (default orange).
            point_size: Size of annotation points in pixels (default 6).
            font_size: Font size for text labels (default 10).
        """
        if self.centerline is None or self.centerline.ss is None:
            return

        # Get centerline data
        ss = self.centerline.ss
        xs = self.centerline.xs
        ys = self.centerline.ys

        # Calculate annotation positions
        # Find arc length values at multiples of interval
        max_s = ss[-1]
        annotation_s_values = np.arange(0, max_s, interval)

        # Interpolate x, y positions at annotation arc lengths
        annotation_points = []
        annotation_labels = []

        for s_val in annotation_s_values:
            # Find closest waypoint index
            idx = np.argmin(np.abs(ss - s_val))
            annotation_points.append([xs[idx], ys[idx]])
            annotation_labels.append(f"{s_val:.1f}m")

        annotation_points = np.array(annotation_points)

        # Render points (with caching for efficiency)
        if self.arc_annotation_points_render is None:
            self.arc_annotation_points_render = e.render_points(annotation_points, color=point_color, size=point_size)
        else:
            # PyQt renderer supports updating via setData
            if hasattr(self.arc_annotation_points_render, "setData"):
                self.arc_annotation_points_render.setData(annotation_points[:, 0], annotation_points[:, 1])
            else:
                # Pygame requires re-rendering
                self.arc_annotation_points_render = e.render_points(
                    annotation_points, color=point_color, size=point_size
                )

        # Render text labels
        # For Pygame: re-render each frame (simple approach)
        # For PyQt: cache TextItem objects if they don't exist yet
        for i, (point, label) in enumerate(zip(annotation_points, annotation_labels)):
            # Offset text slightly to the right and up to avoid overlapping point
            text_pos = (point[0] + 0.3, point[1] + 0.2)

            # For first frame or if not enough cached items, create new text renders
            if len(self.arc_annotation_text_renders) <= i:
                text_render = e.render_text(label, text_pos, color=text_color, font_size=font_size, anchor="left")
                self.arc_annotation_text_renders.append(text_render)
            else:
                # For PyQt, update existing TextItem
                if hasattr(self.arc_annotation_text_renders[i], "setText"):
                    self.arc_annotation_text_renders[i].setText(label)
                    self.arc_annotation_text_renders[i].setPos(text_pos[0], text_pos[1])
                else:
                    # For Pygame, re-render text each frame
                    e.render_text(label, text_pos, color=text_color, font_size=font_size, anchor="left")

    def render_frenet_projection(
        self,
        e: EnvRenderer,
        vehicle_color: tuple[int, int, int] = (255, 0, 255),  # Magenta for vehicle
        projected_color: tuple[int, int, int] = (0, 255, 255),  # Cyan for projected point
        line_color: tuple[int, int, int] = (255, 128, 0),  # Orange for connection line
        size: int = 8,
    ) -> None:
        """Render debug visualisation of Frenet coordinate projection.

        Shows vehicle position (magenta), projected point on centerline (cyan),
        and a connecting line (orange). Requires ``debug_frenet_projection=True``
        in the env config to populate the debug data.

        Args:
            e: Environment renderer object.
            vehicle_color: RGB color for vehicle position marker.
            projected_color: RGB color for projected point marker.
            line_color: RGB color for connection line.
            size: Size of rendered points in pixels.
        """
        if self.debug_vehicle_point is None or self.debug_projected_point is None:
            return  # No data to render yet

        # Render vehicle position
        vehicle_pt = self.debug_vehicle_point.reshape(1, 2)
        if self.debug_render_vehicle is None:
            self.debug_render_vehicle = e.render_points(vehicle_pt, color=vehicle_color, size=size)
        else:
            if hasattr(self.debug_render_vehicle, "setData"):
                self.debug_render_vehicle.setData(vehicle_pt[:, 0], vehicle_pt[:, 1])
            else:
                self.debug_render_vehicle = e.render_points(vehicle_pt, color=vehicle_color, size=size)

        # Render projected point
        projected_pt = self.debug_projected_point.reshape(1, 2)
        if self.debug_render_projected is None:
            self.debug_render_projected = e.render_points(projected_pt, color=projected_color, size=size)
        else:
            if hasattr(self.debug_render_projected, "setData"):
                self.debug_render_projected.setData(projected_pt[:, 0], projected_pt[:, 1])
            else:
                self.debug_render_projected = e.render_points(projected_pt, color=projected_color, size=size)

        # Render connection line (2 points: vehicle and projected)
        line_pts = np.vstack([self.debug_vehicle_point, self.debug_projected_point])
        if self.debug_render_line is None:
            self.debug_render_line = e.render_lines(
                line_pts,
                color=line_color,
                size=2,  # Shape: (2, 2) - two points with x,y coords
            )
        else:
            if hasattr(self.debug_render_line, "setData"):
                self.debug_render_line.setData(line_pts[:, 0], line_pts[:, 1])
            else:
                self.debug_render_line = e.render_lines(line_pts, color=line_color, size=2)

    def render_lookahead_curvatures(
        self,
        e: EnvRenderer,
        vehicle_s: float,
        n_points: int,
        ds: float,
        color: tuple[int, int, int] = (0, 0, 255),
        size: int = 6,
    ) -> None:
        """Render lookahead curvature sampling points ahead of the vehicle.

        Visualises the points where curvature is sampled by the observation
        system, matching the behaviour of :func:`~gymkhana.envs.observation.sample_lookahead_curvatures`.

        Args:
            e: Environment renderer object.
            vehicle_s: Current arc length position of vehicle (meters).
            n_points: Number of lookahead points to render.
            ds: Spacing between points in meters.
            color: RGB color tuple for rendered points.
            size: Size of rendered points in pixels.
        """
        # Check if centerline exists and has spline
        if self.centerline is None or not hasattr(self.centerline, "spline"):
            return  # Early return if no centerline to visualize

        centerline = self.centerline
        track_length = centerline.spline.s[-1]  # Total length for wrap-around

        # Collect points to render
        points = []
        for i in range(n_points):
            # Lookahead arc length (wrap around for closed tracks)
            # Match the formula from observation.py line 58
            s_ahead = (vehicle_s + ((i + 1) * ds)) % track_length

            # Convert s to x,y position
            try:
                x, y = centerline.spline.calc_position(s_ahead)
                points.append([x, y])
            except Exception:
                # Skip invalid points gracefully
                continue

        # Render all valid points
        if len(points) > 0:
            points_array = np.array(points, dtype=np.float32)

            # Handle caching to prevent point accumulation
            if self.lookahead_render is None:
                # First render: create new plot item
                self.lookahead_render = e.render_points(points_array, color=color, size=size)
            else:
                # Update existing plot item using setData()
                # This works for both PyQt (uses setData) and Pygame (returns None, so recreates)
                if hasattr(self.lookahead_render, "setData"):
                    # PyQt renderer: update existing PlotDataItem
                    self.lookahead_render.setData(points_array[:, 0], points_array[:, 1])
                else:
                    # Pygame renderer: returns None, so always recreate
                    self.lookahead_render = e.render_points(points_array, color=color, size=size)
