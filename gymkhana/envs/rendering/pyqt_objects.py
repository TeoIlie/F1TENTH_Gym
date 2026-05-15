from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from numba import njit

from ..collision_models import get_trmtx, get_vertices
from .renderer import RenderSpec


class TextObject:
    """
    Class to display text on the screen at a given position.

    Attributes
    ----------
    font : pygame.font.Font
        font object
    position : str | tuple
        position of the text on the screen
    text : pygame.Surface
        text surface to be displayed
    """

    def __init__(
        self,
        position: str | tuple,
        relative_font_size: int = 16,
        font_name: str = "Arial",
        parent: pg.PlotWidget = None,
    ) -> None:
        """
        Initialize text object.

        Parameters
        ----------
        position : str | tuple
            position of the text on the screen
        relative_font_size : int, optional
            font size relative to the window shape, by default 32
        font_name : str, optional
            font name, by default "Arial"
        """
        self.position = position

        self.text_label = pg.LabelItem(
            "", parent=parent, size=str(relative_font_size) + "pt", family=font_name, color=(125, 125, 125)
        )  # create text label
        # Get the position and offset of the text
        position_tuple = self._position_resolver(self.position)
        offset_tuple = self._offset_resolver(self.position, self.text_label)
        # Set the position and offset of the text
        self.text_label.anchor(itemPos=position_tuple, parentPos=position_tuple, offset=offset_tuple)

    def _position_resolver(self, position: str | tuple[int, int]) -> tuple[int, int]:
        """
        This function takes strings like "bottom center" and converts them into a location for the text to be displayed.
        If position is tuple, then passthrough.

        Parameters
        ----------
        position : str | tuple
            position of the text on the screen

        Returns
        -------
        tuple
            position of the text on the screen

        Raises
        ------
        ValueError
            if position is not a tuple or a string
        NotImplementedError
            if position is a string but not implemented
        """
        if isinstance(position, tuple) and len(position) == 2:
            return int(position[0]), int(position[1])
        elif isinstance(position, str):
            position = position.lower()
            if position == "bottom_right":
                return (1, 1)
            elif position == "bottom_left":
                return (0, 1)
            elif position == "bottom_center":
                return (0.5, 1)
            elif position == "top_right":
                return (1, 0)
            elif position == "top_left":
                return (0, 0)
            elif position == "top_center":
                return (0.5, 0)
            else:
                raise NotImplementedError(f"Position {position} not implemented.")
        else:
            raise ValueError(f"Position expected to be a tuple[int, int] or a string. Got {position}.")

    def _offset_resolver(self, position: str | tuple[int, int], text_label: pg.LabelItem) -> tuple[int, int]:
        """
        This function takes strings like "bottom center" and converts them into a location for the text to be displayed.
        If position is tuple, then passthrough.

        Parameters
        ----------
        position : str | tuple
            position of the text on the screen

        Returns
        -------
        tuple
            position of the text on the screen

        Raises
        ------
        ValueError
            if position is not a tuple or a string
        NotImplementedError
            if position is a string but not implemented
        """
        if isinstance(position, tuple) and len(position) == 2:
            return int(position[0]), int(position[1])
        elif isinstance(position, str):
            position = position.lower()
            if position == "bottom_right":
                return (-text_label.width(), 0)
            elif position == "bottom_left":
                return (0, 0)
            elif position == "bottom_center":
                return (-text_label.width() / 2, 0)
            elif position == "top_right":
                return (-text_label.width(), 0)
            elif position == "top_left":
                return (0, 0)
            elif position == "top_center":
                return (-text_label.width() / 2, 0)
            else:
                raise NotImplementedError(f"Position {position} not implemented.")
        else:
            raise ValueError(f"Position expected to be a tuple[int, int] or a string. Got {position}.")

    def render(self, text: str) -> None:
        """
        Render text on the screen.

        Parameters
        ----------
        text : str
            text to be displayed
        """
        self.text_label.setText(text)


@njit(cache=True)
def _get_tire_vertices(pose, length, width, tire_width, tire_length, index, steering):
    """
    Utility function to return vertices of the car's tire given pose and size

    Args:
        pose (np.ndarray, (3, )): current world coordinate pose of the vehicle
        length (float): car length
        width (float): car width

    Returns:
        vertices (np.ndarray, (4, 2)): corner vertices of the vehicle body
    """
    pose_arr = np.array(pose)
    if index == "fl":
        # Shift back, rotate
        H_shift = get_trmtx(np.array([-(length / 2 - tire_length / 2), -(width / 2 - tire_width / 2), 0]))
        H_steer = get_trmtx(np.array([0, 0, steering]))
        H_back = get_trmtx(np.array([length / 2 - tire_length / 2, width / 2 - tire_width / 2, 0]))
        H = get_trmtx(pose_arr)
        H = H.dot(H_back).dot(H_steer).dot(H_shift)
        fl = H.dot(np.asarray([[length / 2], [width / 2], [0.0], [1.0]])).flatten()
        fr = H.dot(np.asarray([[length / 2], [width / 2 - tire_width], [0.0], [1.0]])).flatten()
        rr = H.dot(np.asarray([[length / 2 - tire_length], [width / 2 - tire_width], [0.0], [1.0]])).flatten()
        rl = H.dot(np.asarray([[length / 2 - tire_length], [width / 2], [0.0], [1.0]])).flatten()
        rl = rl / rl[3]
        rr = rr / rr[3]
        fl = fl / fl[3]
        fr = fr / fr[3]
        vertices = np.asarray([[rl[0], rl[1]], [fl[0], fl[1]], [fr[0], fr[1]], [rr[0], rr[1]], [rl[0], rl[1]]])
    elif index == "fr":
        # Shift back, rotate
        H_shift = get_trmtx(np.array([-(length / 2 - tire_length / 2), -(-width / 2 + tire_width / 2), 0]))
        H_steer = get_trmtx(np.array([0, 0, steering]))
        H_back = get_trmtx(np.array([length / 2 - tire_length / 2, -width / 2 + tire_width / 2, 0]))
        H = get_trmtx(pose_arr)
        H = H.dot(H_back).dot(H_steer).dot(H_shift)

        fl = H.dot(np.asarray([[length / 2], [-width / 2 + tire_width], [0.0], [1.0]])).flatten()
        fr = H.dot(np.asarray([[length / 2], [-width / 2], [0.0], [1.0]])).flatten()
        rr = H.dot(np.asarray([[length / 2 - tire_length], [-width / 2], [0.0], [1.0]])).flatten()
        rl = H.dot(np.asarray([[length / 2 - tire_length], [-width / 2 + tire_width], [0.0], [1.0]])).flatten()
        rl = rl / rl[3]
        rr = rr / rr[3]
        fl = fl / fl[3]
        fr = fr / fr[3]
        # As it is only used for rendering, we can reorder the vertices and append the first point to close the polygon
        vertices = np.asarray([[rl[0], rl[1]], [fl[0], fl[1]], [fr[0], fr[1]], [rr[0], rr[1]], [rl[0], rl[1]]])

    elif index == "rl":
        # Rear-left wheel: no steering. Tire occupies body-frame rectangle:
        # x in [-length/2, -length/2 + tire_length], y in [width/2 - tire_width, width/2].
        H = get_trmtx(pose_arr)
        rl = H.dot(np.asarray([[-length / 2], [width / 2], [0.0], [1.0]])).flatten()
        fl = H.dot(np.asarray([[-length / 2 + tire_length], [width / 2], [0.0], [1.0]])).flatten()
        fr = H.dot(np.asarray([[-length / 2 + tire_length], [width / 2 - tire_width], [0.0], [1.0]])).flatten()
        rr = H.dot(np.asarray([[-length / 2], [width / 2 - tire_width], [0.0], [1.0]])).flatten()
        rl = rl / rl[3]
        fl = fl / fl[3]
        fr = fr / fr[3]
        rr = rr / rr[3]
        vertices = np.asarray([[rl[0], rl[1]], [fl[0], fl[1]], [fr[0], fr[1]], [rr[0], rr[1]], [rl[0], rl[1]]])

    elif index == "rr":
        # Rear-right wheel: no steering.
        H = get_trmtx(pose_arr)
        rl = H.dot(np.asarray([[-length / 2], [-width / 2 + tire_width], [0.0], [1.0]])).flatten()
        fl = H.dot(np.asarray([[-length / 2 + tire_length], [-width / 2 + tire_width], [0.0], [1.0]])).flatten()
        fr = H.dot(np.asarray([[-length / 2 + tire_length], [-width / 2], [0.0], [1.0]])).flatten()
        rr = H.dot(np.asarray([[-length / 2], [-width / 2], [0.0], [1.0]])).flatten()
        rl = rl / rl[3]
        fl = fl / fl[3]
        fr = fr / fr[3]
        rr = rr / rr[3]
        vertices = np.asarray([[rl[0], rl[1]], [fl[0], fl[1]], [fr[0], fr[1]], [rr[0], rr[1]], [rl[0], rl[1]]])

    return vertices


class Car:
    """
    Class to display the car.
    """

    def __init__(
        self,
        render_spec: RenderSpec,
        map_origin: tuple[float, float],
        resolution: float,
        car_length: float,
        car_width: float,
        color: list[int] | None = None,
        wheel_size: float = 0.2,
        parent: pg.PlotWidget = None,
    ):
        self.car_length = car_length
        self.car_width = car_width
        self.wheel_size = wheel_size
        self.car_thickness = render_spec.car_tickness
        self.show_wheels = render_spec.show_wheels

        self.origin = map_origin
        self.resolution = resolution

        self.color = color or (0, 0, 0)
        self.pose = (0, 0, 0)
        self.steering = 0
        self.slip = 0.0
        self.chassis = None

        # Tire params need to be updated
        self.tire_width = 0.1
        self.tire_length = self.wheel_size

        # Only front tires are rendered as polygons (rear polygons were
        # visually redundant). Trails are drawn for all four tires; rear
        # tires don't steer.
        self._tire_indices = ("fl", "fr", "rl", "rr")
        self._wheel_indices = ("fl", "fr")

        # Trail config (slip thresholds stored in radians; yaml is in degrees).
        self.show_trails = render_spec.show_trails
        self.trail_length = int(render_spec.trail_length)
        self.trail_emit_every = max(1, int(render_spec.trail_emit_every))
        self.trail_point_size = int(render_spec.trail_point_size)
        self.trail_max_alpha = int(render_spec.trail_max_alpha)
        self.trail_color = tuple(render_spec.trail_color)
        self.trail_slip_threshold = float(np.deg2rad(render_spec.trail_slip_threshold_deg))
        self.trail_slip_full = float(np.deg2rad(render_spec.trail_slip_full_deg))
        self._trail_slip_span = max(1e-6, self.trail_slip_full - self.trail_slip_threshold)
        self.trail_decay = float(render_spec.trail_decay_per_step)
        self._trail_emit_counter = 0
        # Tracks the previous render's emit state so we can drop a NaN
        # sentinel on the off-edge; PlotCurveItem(connect='finite') then
        # breaks the polyline there, preventing a spurious line segment
        # connecting old and new drift trails across a pause.
        self._emitting_prev = False

        vertices = get_vertices(self.pose, self.car_length, self.car_width)
        # vertices are rl, rr, fr, fl => Reorder to be rl, fl, fr, rr
        vertices = np.array([vertices[0], vertices[3], vertices[2], vertices[1]])
        # Append the first point to close the polygon
        vertices = np.vstack([vertices, vertices[0]])
        self.chassis: pg.PlotDataItem = parent.plot(
            vertices[:, 0],
            vertices[:, 1],
            pen=pg.mkPen(color=(0, 0, 0), width=self.car_thickness),
            fillLevel=0,
            brush=self.color,
        )

        self.wheels: dict[str, pg.PlotDataItem] = {}
        # Per-tire trail state. trail_intensities runs parallel to
        # trail_buffers and stores [0, 1] slip intensity per point so each
        # band can scale its pen alpha by the mean intensity of its slice.
        self.trail_buffers: dict[str, np.ndarray] = {}
        self.trail_intensities: dict[str, np.ndarray] = {}
        self.trail_count: dict[str, int] = {}
        # Two PlotCurveItems per tire (old half + new half). Per-point alpha
        # would force ScatterPlotItem to render N unique symbols/frame and
        # was the original perf killer; banded single-pen polylines are
        # ~20x cheaper and visually equivalent given the time-based decay.
        self.trail_curves: dict[str, tuple[pg.PlotCurveItem, pg.PlotCurveItem]] = {}

        if self.show_wheels:
            for idx in self._wheel_indices:
                verts = _get_tire_vertices(
                    self.pose, self.car_length, self.car_width, self.tire_width, self.tire_length, idx, self.steering
                )
                self.wheels[idx] = parent.plot(
                    verts[:, 0],
                    verts[:, 1],
                    pen=pg.mkPen(color=(0, 0, 0), width=self.car_thickness),
                    fillLevel=0,
                    brush=(0, 0, 0),  # Rubber tire => Black
                )

        # Trail buffers are chronological: oldest at [0], newest at the end.
        # connect='finite' lets NaN slots break the polyline (used for the
        # emit→pause transition). ignoreBounds=True skips trail items in the
        # viewbox auto-range recomputation each frame (significant speedup).
        if self.show_trails:
            empty_pen = pg.mkPen(color=(0, 0, 0, 0))
            for idx in self._tire_indices:
                self.trail_buffers[idx] = np.zeros((self.trail_length, 2), dtype=np.float64)
                self.trail_intensities[idx] = np.zeros(self.trail_length, dtype=np.float64)
                self.trail_count[idx] = 0
                bands = tuple(pg.PlotCurveItem(pen=empty_pen, antialias=False, connect="finite") for _ in range(2))
                for band in bands:
                    parent.addItem(band, ignoreBounds=True)
                self.trail_curves[idx] = bands

    def update(self, state: dict[str, np.ndarray], idx: int):
        self.pose = (
            state["poses_x"][idx],
            state["poses_y"][idx],
            state["poses_theta"][idx],
        )
        self.color = (255, 0, 0) if state["collisions"][idx] > 0 else self.color
        self.steering = state["steering_angles"][idx]
        slip_arr = state.get("slip")
        self.slip = float(slip_arr[idx]) if slip_arr is not None else 0.0

    def render(self):
        vertices = get_vertices(self.pose, self.car_length, self.car_width)
        # vertices are rl, rr, fr, fl => Reorder to be rl, fl, fr, rr
        vertices = np.array([vertices[0], vertices[3], vertices[2], vertices[1]])
        # Append the first point to close the polygon
        vertices = np.vstack([vertices, vertices[0]])

        self.chassis.setData(vertices[:, 0], vertices[:, 1])

        # Compute per-tire vertices once; reused for wheel polygon + trail contact point.
        tire_verts: dict[str, np.ndarray] = {}
        if self.show_wheels or self.show_trails:
            for idx in self._tire_indices:
                steering = self.steering if idx in ("fl", "fr") else 0.0
                tire_verts[idx] = _get_tire_vertices(
                    self.pose, self.car_length, self.car_width, self.tire_width, self.tire_length, idx, steering
                )

        if self.show_wheels:
            for idx in self._wheel_indices:
                verts = tire_verts[idx]
                self.wheels[idx].setData(verts[:, 0], verts[:, 1])

        if self.show_trails:
            self._render_trails(tire_verts)

    def _render_trails(self, tire_verts: dict[str, np.ndarray]) -> None:
        """Update tire-trail buffers and per-band pen alphas for this frame."""
        # Slip-driven intensity in [0, 1]: 0 at threshold, 1 at "full" slip.
        abs_slip = abs(self.slip)
        intensity = (
            min(1.0, (abs_slip - self.trail_slip_threshold) / self._trail_slip_span)
            if abs_slip > self.trail_slip_threshold
            else 0.0
        )
        emit = intensity > 0.0 and (self._trail_emit_counter % self.trail_emit_every) == 0
        self._trail_emit_counter += 1

        # On the first non-emitting frame after an emit run, write a NaN
        # sentinel; otherwise the polyline would connect the old trail to
        # the next one when drifting resumes.
        if emit:
            write_xy, write_int = None, intensity
        elif self._emitting_prev:
            write_xy, write_int = np.nan, 0.0  # NaN sentinel, broken by connect='finite'
        else:
            write_xy = write_int = None  # idle: just decay and re-render existing data
        self._emitting_prev = emit

        r, g, b = self.trail_color
        for idx, verts in tire_verts.items():
            buf = self.trail_buffers[idx]
            inten = self.trail_intensities[idx]
            count = self.trail_count[idx]

            if count > 0:
                inten[:count] *= self.trail_decay

            if write_int is not None:
                # contact ≈ midpoint of the tire's rear edge (verts[0]=rl, verts[3]=rr).
                xy = 0.5 * (verts[0] + verts[3]) if write_xy is None else write_xy
                if count < self.trail_length:
                    buf[count] = xy
                    inten[count] = write_int
                    count += 1
                    self.trail_count[idx] = count
                else:
                    buf[:-1] = buf[1:]
                    inten[:-1] = inten[1:]
                    buf[-1] = xy
                    inten[-1] = write_int

            if count < 2:
                continue

            # Two bands (old half + new half). Pen alpha = mean intensity ×
            # trail_max_alpha, so the heavier the drift the darker the marks.
            # Old band is further attenuated (×0.4) to give the stepped fade.
            old, new = self.trail_curves[idx]
            mid = count // 2
            if mid >= 2:
                self._set_band(old, buf[:mid], inten[:mid], r, g, b, scale=0.4)
            else:
                old.setData(x=[], y=[])
            start_new = max(0, mid - 1)  # overlap by one point so bands visually connect
            self._set_band(new, buf[start_new:count], inten[start_new:count], r, g, b, scale=1.0)

    def _set_band(
        self,
        band: pg.PlotCurveItem,
        pts: np.ndarray,
        intensities: np.ndarray,
        r: int,
        g: int,
        b: int,
        scale: float,
    ) -> None:
        """Set a band's polyline data and pen-alpha proportional to mean intensity."""
        alpha = int(self.trail_max_alpha * float(intensities.mean()) * scale)
        alpha = 0 if alpha < 0 else 255 if alpha > 255 else alpha
        band.setPen(pg.mkPen(color=(r, g, b, alpha), width=self.trail_point_size))
        band.setData(x=pts[:, 0], y=pts[:, 1])
