from __future__ import annotations

import logging
import signal
from typing import Any, Callable, Optional

import numpy as np
import pyqtgraph as pg
from PIL import ImageColor
from PyQt6 import QtCore, QtGui, QtWidgets

from ..track import Track
from .pyqt_objects import (
    Car,
    TextObject,
)
from .renderer import EnvRenderer, RenderSpec

# one-line instructions visualized at the top of the screen (if show_info=True)
INSTRUCTION_TEXT = "Mouse click (L/M/R): Change POV - 'S' key: On/Off"

# control debug panel constants
_DEBUG_PANEL_HEIGHT = 140
_DEBUG_BG_COLOR = QtGui.QColor(26, 26, 26)
_DEBUG_BAR_OUTLINE_COLOR = QtGui.QColor(80, 80, 80)
_DEBUG_ZERO_COLOR = QtGui.QColor(200, 200, 200)
_DEBUG_STEER_COLOR = QtGui.QColor(80, 140, 255)
_DEBUG_THROTTLE_COLOR = QtGui.QColor(80, 200, 120)

# observation debug overlay constants
_OBS_OVERLAY_MARGIN_X = 10
_OBS_OVERLAY_MARGIN_TOP = 45
_OBS_OVERLAY_FONT_SIZE = 10
_OBS_ARRAY_SMALL_THRESHOLD = 15


class ControlDebugPanel(QtWidgets.QWidget):
    """Widget that draws control command text and zero-centered bar gauges."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(_DEBUG_PANEL_HEIGHT)
        self._state = None
        self._font = QtGui.QFont("Monospace", 11)
        self._font.setStyleHint(QtGui.QFont.StyleHint.Monospace)

    def update_state(self, state: dict, agent_idx: int | None, ego_idx: int) -> None:
        idx = agent_idx if agent_idx is not None else ego_idx
        self._state = {
            "steer_cmd": float(state["steering_cmds"][idx]),
            "throttle_cmd": float(state["throttle_cmds"][idx]),
            "v_x": float(state["v_x"][idx]),
            "delta": float(state["delta"][idx]),
            "steer_bounds": state["steer_bounds"],
            "throttle_bounds": state["throttle_bounds"],
            "steer_type": state["steer_type"],
            "throttle_type": state["throttle_type"],
            "delta_bounds": state["delta_bounds"],
            "vx_bounds": state["vx_bounds"],
        }
        self.update()  # schedule repaint

    def paintEvent(self, event):
        if self._state is None:
            return
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), _DEBUG_BG_COLOR)

        # draw 1px separator at top
        p.setPen(QtGui.QPen(_DEBUG_BAR_OUTLINE_COLOR, 1))
        p.drawLine(0, 0, self.width(), 0)

        p.setFont(self._font)
        fm = p.fontMetrics()
        lh = fm.height() + 2  # line height
        margin = 12
        s = self._state

        # Row 1: actual state in white — delta left-aligned, v_x right-aligned
        y = margin + fm.ascent()
        delta_text = f"delta = {s['delta']:+.4f} rad  [{s['delta_bounds'][0]:.2f}, {s['delta_bounds'][1]:.2f}]"
        vx_text = f"v_x = {s['v_x']:+.3f} m/s  [{s['vx_bounds'][0]:.2f}, {s['vx_bounds'][1]:.2f}]"
        p.setPen(QtGui.QColor(255, 255, 255))
        p.drawText(margin, y, delta_text)
        p.drawText(self.width() - margin - fm.horizontalAdvance(vx_text), y, vx_text)

        # Row 2: command text — steer left-aligned in steer color, throttle right-aligned in throttle color
        y += lh
        steer_text = (
            f"{s['steer_type']} cmd: {s['steer_cmd']:+.3f}  [{s['steer_bounds'][0]:.2f}, {s['steer_bounds'][1]:.2f}]"
        )
        throttle_text = f"{s['throttle_type']} cmd: {s['throttle_cmd']:+.3f}  [{s['throttle_bounds'][0]:.2f}, {s['throttle_bounds'][1]:.2f}]"
        p.setPen(_DEBUG_STEER_COLOR)
        p.drawText(margin, y, steer_text)
        p.setPen(_DEBUG_THROTTLE_COLOR)
        p.drawText(self.width() - margin - fm.horizontalAdvance(throttle_text), y, throttle_text)

        # Row 3-4: bars (no text labels — colour matches the text above)
        min_label_w = fm.horizontalAdvance("-00.00 ") + 4
        bar_x = margin + min_label_w
        bar_w = self.width() - 2 * (margin + min_label_w)
        bar_h = 16
        y_bar = y + lh

        self._draw_bar(p, fm, s["steer_cmd"], s["steer_bounds"], _DEBUG_STEER_COLOR, bar_x, bar_w, bar_h, y_bar)
        y_bar += bar_h + 10
        self._draw_bar(
            p, fm, s["throttle_cmd"], s["throttle_bounds"], _DEBUG_THROTTLE_COLOR, bar_x, bar_w, bar_h, y_bar
        )

        p.end()

    def _draw_bar(
        self,
        p: QtGui.QPainter,
        fm,
        value: float,
        bounds: tuple,
        color: QtGui.QColor,
        bar_x: int,
        bar_w: int,
        bar_h: int,
        y: int,
    ):
        lo, hi = bounds
        rng = hi - lo
        if rng <= 0:
            return

        # outline
        p.setPen(QtGui.QPen(_DEBUG_BAR_OUTLINE_COLOR, 1))
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        p.drawRect(bar_x, y, bar_w, bar_h)

        zero_x = bar_x + int((0.0 - lo) / rng * bar_w)
        val_x = bar_x + int((max(lo, min(hi, value)) - lo) / rng * bar_w)

        # fill from zero to value
        fill_left, fill_right = min(zero_x, val_x), max(zero_x, val_x)
        if fill_right > fill_left:
            p.fillRect(fill_left, y + 1, fill_right - fill_left, bar_h - 1, color)

        # zero tick
        p.setPen(QtGui.QPen(_DEBUG_ZERO_COLOR, 2))
        p.drawLine(zero_x, y - 2, zero_x, y + bar_h + 2)

        # min/max labels
        p.drawText(bar_x - fm.horizontalAdvance(f"{lo:.2f}") - 2, y + fm.ascent(), f"{lo:.2f}")
        p.drawText(bar_x + bar_w + 4, y + fm.ascent(), f"{hi:.2f}")

        # value marker
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255), 2))
        p.drawLine(val_x, y, val_x, y + bar_h)


class ObsDebugOverlay(QtWidgets.QWidget):
    """Widget that overlays observation key-value text on top of the map."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setStyleSheet("background: transparent;")
        self._lines: list[str] = []
        self._font = QtGui.QFont("Monospace", _OBS_OVERLAY_FONT_SIZE)
        self._font.setStyleHint(QtGui.QFont.StyleHint.Monospace)

    def update_state(self, state: dict, agent_idx: int | None, ego_idx: int) -> None:
        get_features = state.get("obs_debug_getter")
        if get_features is None:
            self._lines = ["[no observation data]"]
            self.update()
            return

        normalize = state.get("obs_debug_normalize", False)
        idx = agent_idx if agent_idx is not None else ego_idx

        lines = [f"[norm: {'on' if normalize else 'off'}]"]

        agent_features = get_features(idx)
        for key, value in agent_features.items():
            lines.extend(self._format_feature(key, value))

        self._lines = lines
        self.update()

    def _format_feature(self, key: str, value) -> list[str]:
        if isinstance(value, np.ndarray):
            if value.size > _OBS_ARRAY_SMALL_THRESHOLD:
                return [f"{key}: [{value.size}] min={value.min():.3f} max={value.max():.3f} mean={value.mean():.3f}"]
            elif value.size > 1:
                formatted = ", ".join(f"{v:.3f}" for v in value.flat)
                return [f"{key}: [{formatted}]"]
            else:
                return [f"{key}: {float(value):.4f}"]
        elif isinstance(value, (int, np.integer)):
            return [f"{key}: {value}"]
        elif isinstance(value, (float, np.floating)):
            return [f"{key}: {value:.4f}"]
        else:
            return [f"{key}: {value}"]

    def paintEvent(self, event):
        if not self._lines:
            return
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.TextAntialiasing)
        p.setFont(self._font)
        fm = p.fontMetrics()
        lh = fm.height() + 2
        pad = 6

        # compute bounding rect for all lines
        max_width = max(fm.horizontalAdvance(line) for line in self._lines)
        total_height = lh * len(self._lines)
        bg_rect = QtCore.QRect(
            _OBS_OVERLAY_MARGIN_X - pad,
            _OBS_OVERLAY_MARGIN_TOP - pad,
            max_width + 2 * pad,
            total_height + 2 * pad,
        )

        # semi-transparent dark background
        p.fillRect(bg_rect, QtGui.QColor(0, 0, 0, 150))

        # white text
        p.setPen(QtGui.QColor(255, 255, 255))
        x = _OBS_OVERLAY_MARGIN_X
        y = _OBS_OVERLAY_MARGIN_TOP + fm.ascent()
        for line in self._lines:
            p.drawText(x, y, line)
            y += lh

        p.end()


# Replicated from pyqtgraphs' example utils for ci pipelines to pass
from time import perf_counter


class FrameCounter(QtCore.QObject):
    sigFpsUpdate = QtCore.pyqtSignal(object)

    def __init__(self, interval=1000):
        super().__init__()
        self.count = 0
        self.last_update = 0
        self.interval = interval

    def update(self):
        self.count += 1

        if self.last_update == 0:
            self.last_update = perf_counter()
            self.startTimer(self.interval)

    def timerEvent(self, evt):
        now = perf_counter()
        elapsed = now - self.last_update
        fps = self.count / elapsed
        self.last_update = now
        self.count = 0
        self.sigFpsUpdate.emit(fps)


class PyQtEnvRenderer(EnvRenderer):
    """
    Renderer of the environment using PyQtGraph.
    """

    def __init__(
        self,
        params: dict[str, Any],
        track: Track,
        agent_ids: list[str],
        render_spec: RenderSpec,
        render_mode: str,
        render_fps: int,
    ):
        """
        Initialize the Pygame renderer.

        Parameters
        ----------
        params : dict
            dictionary of simulation parameters (including vehicle dimensions, etc.)
        track : Track
            track object
        agent_ids : list
            list of agent ids to render
        render_spec : RenderSpec
            rendering specification
        render_mode : str
            rendering mode in ["human", "human_fast", "rgb_array"]
        render_fps : int
            number of frames per second
        """
        super().__init__()
        self.params = params
        self.agent_ids = agent_ids

        self.cars = None
        self.sim_time = None
        self.window = None
        self.canvas = None

        self.render_spec = render_spec
        self.render_mode = render_mode
        self.render_fps = render_fps

        # create the canvas
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

        self.debug_panel = None
        self.obs_overlay = None
        self._plot_widget = pg.GraphicsLayoutWidget()
        ws = self.render_spec.window_size

        if self.render_spec.show_ctr_debug:
            self.window = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(self.window)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            layout.addWidget(self._plot_widget)
            self.debug_panel = ControlDebugPanel()
            layout.addWidget(self.debug_panel)
            self.window.setGeometry(0, 0, ws, ws + _DEBUG_PANEL_HEIGHT)
        else:
            self.window = self._plot_widget
            self.window.setGeometry(0, 0, ws, ws)

        if self.render_spec.show_obs_debug:
            self.obs_overlay = ObsDebugOverlay(parent=self._plot_widget)
            self.obs_overlay.setGeometry(0, 0, ws, ws)
            self.obs_overlay.raise_()

        self.window.setWindowTitle("F1Tenth Gym")
        self.canvas: pg.PlotItem = self._plot_widget.addPlot()

        # Disable interactivity
        self.canvas.setMouseEnabled(x=False, y=False)  # Disable mouse panning & zooming
        self.canvas.hideButtons()  # Disable corner auto-scale button
        self.canvas.setMenuEnabled(False)  # Disable right-click context menu

        legend = self.canvas.addLegend()  # This doesn't disable legend interaction
        # Override both methods responsible for mouse events
        legend.mouseDragEvent = lambda *args, **kwargs: None
        legend.hoverEvent = lambda *args, **kwargs: None
        # self.scene() is a pyqtgraph.GraphicsScene.GraphicsScene.GraphicsScene
        self._plot_widget.scene().sigMouseClicked.connect(self.mouse_clicked)
        self.window.keyPressEvent = self.key_pressed

        # Remove axes
        self.canvas.hideAxis("bottom")
        self.canvas.hideAxis("left")

        # Lock aspect ratio so the map is not skewed when debug panel is shown
        self.canvas.setAspectLocked(True)

        # setting plot window background color to white
        self._plot_widget.setBackground("w")

        # fps and time renderer
        self.clock = FrameCounter()
        self.fps_renderer = TextObject(parent=self.canvas, position="bottom_left")
        self.time_renderer = TextObject(parent=self.canvas, position="bottom_right")
        self.bottom_info_renderer = TextObject(parent=self.canvas, position="bottom_center")
        self.top_info_renderer = TextObject(parent=self.canvas, position="top_center")

        if self.render_mode in ["human", "human_fast"]:
            self.clock.sigFpsUpdate.connect(lambda fps: self.fps_renderer.render(f"FPS: {fps:.1f}"))

        colors_rgb = [[rgb for rgb in ImageColor.getcolor(c, "RGB")] for c in render_spec.vehicle_palette]
        self.car_colors = [colors_rgb[i % len(colors_rgb)] for i in range(len(self.agent_ids))]

        width, height = render_spec.window_size, render_spec.window_size

        # map metadata
        self.map_origin = track.spec.origin
        self.map_resolution = track.spec.resolution

        # load map image
        original_img = track.occupancy_map

        # convert shape from (W, H) to (W, H, 3)
        track_map = np.stack([original_img, original_img, original_img], axis=-1)

        # rotate and flip to match the track orientation
        track_map = np.rot90(track_map, k=1)  # rotate clockwise
        track_map = np.flip(track_map, axis=0)  # flip vertically

        self.image_item = pg.ImageItem(track_map)
        # Example: Transformed display of ImageItem
        tr = QtGui.QTransform()  # prepare ImageItem transformation:
        # Translate image by the origin of the map
        tr.translate(self.map_origin[0], self.map_origin[1])
        # Scale image by the resolution of the map
        tr.scale(self.map_resolution, self.map_resolution)
        self.image_item.setTransform(tr)
        self.canvas.addItem(self.image_item)

        # callbacks for custom visualization, called at every rendering step
        self.callbacks = []

        # event handling flags
        self.draw_flag: bool = True
        if render_spec.focus_on:
            self.active_map_renderer = "car"
            self.follow_agent_flag: bool = True
            self.agent_to_follow: int = self.agent_ids.index(render_spec.focus_on)
        else:
            self.active_map_renderer = "map"
            self.follow_agent_flag: bool = False
            self.agent_to_follow: int = None

        if self.render_mode in ["human", "human_fast"]:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            self.window.show()
        elif self.render_mode == "rgb_array":
            pass  # rgb_array captured via QWidget.grab() in render()

    def update(self, state: dict) -> None:
        """
        Update the simulation state to be rendered.

        Parameters
        ----------
            state: simulation state as dictionary
        """
        if self.cars is None:
            self.cars = [
                Car(
                    car_length=self.params["length"],
                    car_width=self.params["width"],
                    color=self.car_colors[ic],
                    render_spec=self.render_spec,
                    map_origin=self.map_origin[:2],
                    resolution=self.map_resolution,
                    parent=self.canvas,
                )
                for ic in range(len(self.agent_ids))
            ]

        # update cars state and zoom level (updating points-per-unit)
        for i in range(len(self.agent_ids)):
            self.cars[i].update(state, i)

        # update time
        self.sim_time = state["sim_time"]

        # update debug panel
        if self.debug_panel is not None and "steering_cmds" in state:
            self.debug_panel.update_state(state, self.agent_to_follow, state.get("ego_idx", 0))

        # update observation overlay
        if self.obs_overlay is not None and "obs_debug_getter" in state:
            self.obs_overlay.update_state(state, self.agent_to_follow, state.get("ego_idx", 0))

    def add_renderer_callback(self, callback_fn: Callable[[EnvRenderer], None]) -> None:
        """
        Add a custom callback for visualization.
        All the callbacks are called at every rendering step, after having rendered the map and the cars.

        Parameters
        ----------
        callback_fn : Callable[[EnvRenderer], None]
            callback function to be called at every rendering step
        """
        self.callbacks.append(callback_fn)

    def key_pressed(self, event: QtGui.QKeyEvent) -> None:
        """
        Handle key press events.

        Parameters
        ----------
        event : QtGui.QKeyEvent
            key event
        """
        if event.key() == QtCore.Qt.Key.Key_S:
            logging.debug("Pressed S key -> Enable/disable rendering")
            self.draw_flag = not self.draw_flag
            self.draw_flag_changed = True

    def mouse_clicked(self, event: QtGui.QMouseEvent) -> None:
        """
        Handle mouse click events.

        Parameters
        ----------
        event : QtGui.QMouseEvent
            mouse event
        """
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            logging.debug("Pressed left button -> Follow Next agent")

            self.follow_agent_flag = True
            if self.agent_to_follow is None:
                self.agent_to_follow = 0
            else:
                self.agent_to_follow = (self.agent_to_follow + 1) % len(self.agent_ids)

            self.active_map_renderer = "car"
        elif event.button() == QtCore.Qt.MouseButton.RightButton:
            logging.debug("Pressed right button -> Follow Previous agent")

            self.follow_agent_flag = True
            if self.agent_to_follow is None:
                self.agent_to_follow = 0
            else:
                self.agent_to_follow = (self.agent_to_follow - 1) % len(self.agent_ids)

            self.active_map_renderer = "car"
        elif event.button() == QtCore.Qt.MouseButton.MiddleButton:
            logging.debug("Pressed middle button -> Change to Map View")

            self.follow_agent_flag = False
            self.agent_to_follow = None

            self.active_map_renderer = "map"

    def render(self) -> Optional[np.ndarray]:
        """
        Render the current state in a frame.
        It renders in the order: map, cars, callbacks, info text.

        Returns
        -------
        Optional[np.ndarray]
            if render_mode is "rgb_array", returns the rendered frame as an array
        """
        # draw cars
        for i in range(len(self.agent_ids)):
            self.cars[i].render()

        # call callbacks
        for callback_fn in self.callbacks:
            callback_fn(self)

        if self.follow_agent_flag:
            ego_x, ego_y = self.cars[self.agent_to_follow].pose[:2]
            self.canvas.setXRange(ego_x - 10, ego_x + 10)
            self.canvas.setYRange(ego_y - 10, ego_y + 10)
        else:
            self.canvas.autoRange()

        agent_to_follow_id = self.agent_ids[self.agent_to_follow] if self.agent_to_follow is not None else None
        self.bottom_info_renderer.render(text=f"Focus on: {agent_to_follow_id}")

        if self.render_spec.show_info:
            self.top_info_renderer.render(text=INSTRUCTION_TEXT)

        self.time_renderer.render(text=f"{self.sim_time:.2f}")
        self.clock.update()

        if self.obs_overlay is not None:
            self.obs_overlay.setGeometry(0, 0, self._plot_widget.width(), self._plot_widget.height())

        self.app.processEvents()

        if self.render_mode in ["human", "human_fast"]:
            assert self.window is not None

        else:
            # rgb_array mode => grab the whole window so debug widgets are included
            pixmap = self.window.grab()
            qImage = pixmap.toImage().convertToFormat(QtGui.QImage.Format.Format_RGBA8888)

            width = qImage.width()
            height = qImage.height()

            ptr = qImage.bits()
            ptr.setsize(height * width * 4)
            frame = np.array(ptr).reshape(height, width, 4)  # Copies the data

            return frame[:, :, :3]  # remove alpha channel

    def render_points(
        self,
        points: list | np.ndarray,
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
    ) -> pg.PlotDataItem:
        """
        Render a sequence of xy points on screen.

        Parameters
        ----------
        points : list | np.ndarray
            list of points to render
        color : Optional[tuple[int, int, int]], optional
            color as rgb tuple, by default blue (0, 0, 255)
        size : Optional[int], optional
            size of the points in pixels, by default 1
        """
        return self.canvas.plot(
            points[:, 0],
            points[:, 1],
            pen=None,
            symbol="o",
            symbolPen=pg.mkPen(color=color, width=0),
            symbolBrush=pg.mkBrush(color=color, width=0),
            symbolSize=size,
        )

    def render_text(
        self,
        text: str,
        position: tuple[float, float],
        color: Optional[tuple[int, int, int]] = (255, 255, 255),
        font_size: Optional[int] = 12,
        anchor: Optional[str] = "center",
    ) -> pg.TextItem:
        """
        Render text at world coordinates using PyQtGraph TextItem.

        Parameters
        ----------
        text : str
            text string to render
        position : tuple[float, float]
            world coordinate position (x, y) for text placement
        color : tuple[int, int, int], optional
            RGB color tuple, by default white (255, 255, 255)
        font_size : int, optional
            font size in points, by default 12
        anchor : str, optional
            text anchor point ('center', 'left', 'right'), by default 'center'

        Returns
        -------
        pg.TextItem
            the created text item object
        """
        # Convert anchor string to tuple for PyQtGraph
        if anchor == "center":
            anchor_tuple = (0.5, 0.5)
        elif anchor == "left":
            anchor_tuple = (0, 0.5)
        elif anchor == "right":
            anchor_tuple = (1.0, 0.5)
        else:
            anchor_tuple = (0.5, 0.5)

        # Create text item
        text_item = pg.TextItem(text, color=color, anchor=anchor_tuple)
        text_item.setFont(QtGui.QFont("Arial", font_size))
        text_item.setPos(position[0], position[1])
        self.canvas.addItem(text_item)
        return text_item

    def render_lines(
        self,
        points: list | np.ndarray,
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
    ) -> pg.PlotDataItem:
        """
        Render a sequence of lines segments.

        Parameters
        ----------
        points : list | np.ndarray
            list of points to render
        color : Optional[tuple[int, int, int]], optional
            color as rgb tuple, by default blue (0, 0, 255)
        size : Optional[int], optional
            size of the line, by default 1
        """
        pen = pg.mkPen(color=pg.mkColor(*color), width=size)
        return self.canvas.plot(
            points[:, 0], points[:, 1], pen=pen, fillLevel=None, antialias=True
        )  ## setting pen=None disables line drawing

    def render_closed_lines(
        self,
        points: list | np.ndarray,
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
    ) -> pg.PlotDataItem:
        """
        Render a sequence of lines segments forming a closed loop (draw a line between the last and the first point).

        Parameters
        ----------
        points : list | np.ndarray
            list of 2d points to render
        color : Optional[tuple[int, int, int]], optional
            color as rgb tuple, by default blue (0, 0, 255)
        size : Optional[int], optional
            size of the line, by default 1
        """
        # Append the first point to the end to close the loop
        points = np.vstack([points, points[0]])

        pen = pg.mkPen(color=pg.mkColor(*color), width=size)
        pen.setCapStyle(pg.QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(pg.QtCore.Qt.PenJoinStyle.RoundJoin)

        return self.canvas.plot(
            points[:, 0], points[:, 1], pen=pen, cosmetic=True, antialias=True
        )  ## setting pen=None disables line drawing

    def close(self) -> None:
        """
        Close the rendering environment.
        """
        self.app.exit()
