from __future__ import annotations

import pathlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import yaml

# default output directory for frames saved via EnvRenderer.save_frame(), relative to the cwd
SCREENSHOT_DIR = pathlib.Path("figures") / "frames"


@dataclass
class RenderSpec:
    window_size: int
    zoom_in_factor: float
    focus_on: str
    car_tickness: int
    show_wheels: bool
    show_info: Optional[bool] = True
    show_ctr_debug: Optional[bool] = False
    show_obs_debug: Optional[bool] = False
    vehicle_palette: Optional[list[str]] = None
    render_type: Optional[str] = "pygame"
    screenshot_scale: Optional[int] = 4

    def __init__(
        self,
        window_size: int = 800,
        focus_on: str = None,
        zoom_in_factor: float = 1.0,
        car_tickness: int = 1,
        show_wheels: bool = False,
        show_info: bool = True,
        show_ctr_debug: bool = False,
        show_obs_debug: bool = False,
        vehicle_palette: list[str] = None,
        render_type: str = "pygame",
        screenshot_scale: int = 4,
    ) -> None:
        """
        Initialize rendering specification.

        Parameters
        ----------
        window_size : int, optional
            size of the square window, by default 800
        focus_on : str, optional
            focus on a specific vehicle, by default None
        zoom_in_factor : float, optional
            zoom in factor, by default 1.0 (no zoom)
        car_tickness : int, optional
            thickness of the car in pixels, by default 1
        show_wheels : bool, optional
            toggle rendering of line segments for wheels, by default False
        show_info : bool, optional
            toggle rendering of text instructions, by default True
        vehicle_palette : list, optional
            list of colors for rendering vehicles according to their id, by default None
        screenshot_scale : int, optional
            resolution multiplier over the window size for frames saved via
            :meth:`EnvRenderer.save_frame`, by default 4
        """
        self.window_size = window_size
        self.focus_on = focus_on
        self.zoom_in_factor = zoom_in_factor
        self.car_tickness = car_tickness
        self.show_wheels = show_wheels
        self.show_info = show_info
        self.show_ctr_debug = show_ctr_debug
        self.show_obs_debug = show_obs_debug
        self.vehicle_palette = vehicle_palette or ["#984ea3"]
        self.render_type = render_type
        self.screenshot_scale = screenshot_scale

    @staticmethod
    def from_yaml(yaml_file: str | pathlib.Path, overrides: Optional[dict] = None):
        """
        Load rendering specification from a yaml file, optionally overriding fields.

        Parameters
        ----------
        yaml_file : str | pathlib.Path
            path to the yaml file
        overrides : dict, optional
            dict of field values that take precedence over the yaml contents

        Returns
        -------
        RenderSpec
            rendering specification object
        """
        with open(yaml_file, "r") as yaml_stream:
            try:
                config = yaml.safe_load(yaml_stream)
            except yaml.YAMLError as ex:
                raise ValueError(f"Failed to parse render config YAML at {yaml_file}: {ex}") from ex
        if overrides:
            config.update(overrides)
        return RenderSpec(**config)


class EnvRenderer(ABC):
    """
    Abstract class for rendering the environment.
    """

    @abstractmethod
    def update(self, state: Any) -> None:
        """
        Update the state to be rendered.
        This is called at every rendering call.

        Parameters
        ----------
        state : Any
            state to be rendered, e.g. a list of vehicle states
        """
        raise NotImplementedError()

    @abstractmethod
    def render(self):
        """
        Render the current state in a frame.
        """
        raise NotImplementedError()

    @abstractmethod
    def render_lines(
        self,
        points: list | np.ndarray,
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
    ):
        """
        Render a sequence of lines segments.

        Parameters
        ----------
        points : list | np.ndarray
            list of points to render
        color : tuple[int, int, int], optional
            color as rgb tuple, by default blue (0, 0, 255)
        size : int, optional
            size of the line, by default 1
        """
        raise NotImplementedError()

    @abstractmethod
    def render_closed_lines(
        self,
        points: list | np.ndarray,
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
    ):
        """
        Render a closed loop of lines (draw a line between the last and the first point).

        Parameters
        ----------
        points : list | np.ndarray
            list of points to render
        color : tuple[int, int, int], optional
            color as rgb tuple, by default blue (0, 0, 255)
        size : int, optional
            size of the line, by default 1
        """
        raise NotImplementedError()

    @abstractmethod
    def render_text(
        self,
        text: str,
        position: tuple[float, float],
        color: Optional[tuple[int, int, int]] = (255, 255, 255),
        font_size: Optional[int] = 12,
        anchor: Optional[str] = "center",
    ):
        """
        Render text at world coordinates.

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
        """
        raise NotImplementedError()

    def save_frame(self, path: Optional[str] = None, scale: Optional[int] = None) -> str:
        """
        Save the currently rendered frame to an image file.

        Parameters
        ----------
        path : str, optional
            output file path. By default a timestamped png under ``figures/frames/``.
        scale : int, optional
            resolution multiplier over the on-screen window size. By default the
            ``screenshot_scale`` field of the rendering spec.

        Returns
        -------
        str
            path of the written file
        """
        raise NotImplementedError()

    def resolve_frame_path(self, path: Optional[str] = None) -> pathlib.Path:
        """
        Build the output path for :meth:`save_frame`, creating parent directories.

        A default name is derived from the current sim time. Existing files are never
        overwritten: a ``-1``, ``-2``, ... suffix is appended on collision, so frames
        captured at the same sim time (e.g. across episode resets) do not clobber each other.

        Parameters
        ----------
        path : str, optional
            requested output path, or None to use the default location and name

        Returns
        -------
        pathlib.Path
            a path that does not yet exist, with its parent directory created
        """
        if path is not None:
            out_path = pathlib.Path(path)
        else:
            # sim_time is None until the first update() call
            sim_time = getattr(self, "sim_time", None) or 0.0
            out_path = SCREENSHOT_DIR / f"frame_t{sim_time:07.2f}s.png"

        out_path.parent.mkdir(parents=True, exist_ok=True)

        stem, suffix, index = out_path.stem, out_path.suffix, 1
        while out_path.exists():
            out_path = out_path.with_name(f"{stem}-{index}{suffix}")
            index += 1

        return out_path

    @abstractmethod
    def close(self):
        """
        Close the rendering window.
        """
        raise NotImplementedError()
