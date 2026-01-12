from dash import Dash, html, dcc, Output, Input, callback, no_update
from typing import List, Tuple, Any, Dict
import pandas as pd
import plotly.graph_objects as go
import json
import numpy as np
import os
import argparse
import platform
import base64, io
from pathlib import Path

# Load Base Route CSV
REQUIRED_COLUMNS = ["lat", "lon", "alt"]
STRAT_ROOT: Path

class App:
    _DISPLAY_STYLE = {"display": "block", "textAlign": "center"}
    _HIDE_STYLE = {"display": "none"}
    _INVALID_LOOP_MSG = "Invalid loop selection. "
    _INVALID_PLAN_MSG = "Invalid Plan selected, reason: "

    def __init__(self, base_route_path: Path):
        self._base_route_points = App.load_base_route(base_route_path)
        if len(self._base_route_points.shape) < 2:
            raise RuntimeError("Route points must have 2 columns")
        if self._base_route_points.shape[1] < 2:
            raise RuntimeError("Route points must have 2 columns")

        self._race_plan: List[List[Any]]  # List of list of json objects
        self._app = Dash()
        self._setup_layout()
        self._setup_callbacks()
        self._app.run(debug=True)

    @staticmethod
    def load_base_route(base_route_path: Path) -> np.ndarray:
        if not base_route_path.exists():
            raise RuntimeError(f"Base Route {str(base_route_path)} does not exist!")

        df = pd.read_csv(base_route_path, header=None, names=REQUIRED_COLUMNS)

        # Validate presence of required columns
        if df.shape[1] < len(REQUIRED_COLUMNS):
            raise ValueError(f"CSV is missing required columns: expected {REQUIRED_COLUMNS}, got {list(df.columns)}")

        # Validate non-empty and numeric data
        if df.isnull().any().any():
            raise ValueError("CSV contains missing (NaN) values.")

        if not all(np.issubdtype(df[col].dtype, np.number) for col in ["lat", "lon"]):
            raise TypeError("Latitude and longitude columns must be numeric.")

        if df.empty:
            raise ValueError("CSV is empty—no points to load.")

        lat = df["lat"].values
        lon = df["lon"].values
        points = np.column_stack((lon, lat))

        return points

    def _setup_layout(self):
        """Setup app layout and default route"""
        # Dash Layout
        self._app.layout = html.Div(
            [
                dcc.Store(id="race_plan_json"),
                dcc.Store(id="race_plan_metadata"),
                # Header bar layout | Race Plan JSON Upload | Loop Number | Race Plan Validity
                html.Div(
                    children=[
                        # Choose JSON file
                        html.Label("Race Plan JSON File:"),
                        dcc.Upload(
                            id='upload-json',
                            children=html.Button("Upload Race Plan JSON"),
                            multiple=False,
                            style={"padding": "10px", "border": "1px solid black", "cursor": "pointer"}
                        ),
                        # Choose Loop
                        html.Label("Loop:"),
                        dcc.Input(
                            id="loop",
                            value="1",
                            type="number",
                            style={"width": "60px"},
                        ),
                        # Error/Warning messages
                        html.H3(id="msg")
                    ],
                    style={
                        "display": "flex",
                        "justifyContent": "center",
                        "alignItems": "center",
                        "gap": "15px",
                        "margin": "20px",
                    },
                ),
                # Graph to display the route
                dcc.Graph(
                    id="graph",
                    figure=go.Figure(),
                    style={"border": "1px solid", "borderRadius": "10px"},
                ),
            ]
        )

    def _setup_callbacks(self):
        """Add callbacks from selecting a new race plan .json file and the loop number"""

        # Callback for selecting a race plan .json file
        @callback(
            Output("race_plan_json", "data"),
            Output("race_plan_metadata", "data"),
            Output("msg", "children", allow_duplicate = True),
            Output("msg", "style", allow_duplicate = True),
            Input("upload-json", "contents"),
            prevent_initial_call = True
        )
        def load_json(contents):
            if contents is None:
                return no_update

            # Dash stores as base64 encoded string
            try:
                content_type, content_string = contents.split(',')
                decoded = base64.b64decode(content_string)
                self._race_plan = json.load(io.StringIO(decoded.decode('utf-8')))
                if len(self._race_plan) <= 1:
                    invalid_race_plan_message = self._INVALID_PLAN_MSG + \
                                "Race Plan JSON should have at least two elements - [metadata, loops...]"
                    return no_update, no_update, invalid_race_plan_message, self._DISPLAY_STYLE
                if not isinstance(self._race_plan[0], dict):
                    invalid_race_plan_message = self._INVALID_PLAN_MSG + \
                                "Race Plan JSON first element should be a dictionary with metadata"
                    return no_update, no_update, invalid_race_plan_message, self._DISPLAY_STYLE
            except Exception as e:
                error_msg = "ERROR: Could not parse race plan .json file " + e.msg
                return no_update, no_update, error_msg, self._DISPLAY_STYLE

            # Re-construct such that it can be rendered
            race_plan_render = []
            metadata = self._race_plan[0]
            for loop in self._race_plan[1:]:
                route_idx = 0
                loop_segments = []

                # For each segment in the loop
                for seg in range(len(self._base_route_points) - 1):
                    if route_idx >= len(loop):
                        break

                    x_coords = [self._base_route_points[seg][0], self._base_route_points[seg + 1][0]]
                    y_coords = [self._base_route_points[seg][1], self._base_route_points[seg + 1][1]]
                    seg_start, seg_end = (
                        loop[route_idx]["start_idx"],
                        loop[route_idx]["end_idx"],
                    )
                    speed_start, speed_end = (
                        loop[route_idx]["start_speed"],
                        loop[route_idx]["end_speed"],
                    )
                    accel = loop[route_idx]["acceleration_value"]

                    if seg == seg_end:
                        route_idx += 1

                    line_color = "black"
                    if accel < 0.0:
                        line_color = "red"
                    elif accel > 0.0:
                        line_color = "green"

                    loop_segments.append(
                        {
                            "type": "scatter",
                            "x": x_coords,
                            "y": y_coords,
                            "mode": "lines",
                            "line": {"color": line_color, "width": 4},
                            "hoverinfo": "text",
                            "text": f"""Segment {seg_start} to {seg_end}<br>
                            Speed: {speed_start:.2f} to {speed_end:.2f} kp/h<br>
                            Acceleration: {accel:.2f} m/s²""",
                        }
                    )

                race_plan_render.append(loop_segments)

            return race_plan_render, metadata, "", self._HIDE_STYLE

        @callback(
            Output("graph", "figure"),
            Output("msg", "children", allow_duplicate=True),
            Output("msg", "style", allow_duplicate=True),
            Input("loop", "value"),
            Input("race_plan_json", "data"),
            Input("race_plan_metadata", "data"),
            prevent_initial_call=True
        )
        def update_graph(loop, race_plan_data, race_plan_metadata):
            """Update the graph displaying the track route"""
            figure = go.Figure()

            if not race_plan_data or not race_plan_metadata:
                return no_update, "", self._HIDE_STYLE

            # Validate loop
            try:
                loop_index = int(loop) - 1
                if loop_index < 0 or loop_index >= len(race_plan_data):
                    invalid_loop_msg = (
                        self._INVALID_LOOP_MSG + f"Valid range is [1,{len(race_plan_data)}]"
                    )
                    return no_update, invalid_loop_msg, self._DISPLAY_STYLE

                # Construct metadata annotation
                metadata_str = "Race Plan Metadata:<br>"
                metadata_str += "<br>".join(
                    f"{str(key)}: {str(value)}" for key, value in race_plan_metadata.items()
                )

                # Add trace(s) from selected loop
                figure.add_traces(race_plan_data[loop_index])

                # Configure layout and metadata box
                figure.update_layout(
                    xaxis=dict(title="Longitude"),
                    yaxis=dict(title="Latitude"),
                    showlegend=False,
                    height=700,
                    annotations=[
                        dict(
                            text=metadata_str,
                            xref="paper", yref="paper",
                            x=0.01, y=0.99,
                            xanchor="left", yanchor="top",
                            showarrow=False,
                            align="left",
                            font=dict(size=12, color="black"),
                            bgcolor="rgba(255,255,255,0.85)",
                            bordercolor="black",
                            borderwidth=1,
                            borderpad=4,
                        )
                    ]
                )

                return figure, "", self._HIDE_STYLE

            except (ValueError, TypeError, IndexError):
                return figure, "Generic Error Encountered", self._DISPLAY_STYLE

def resolve_path(path: str) -> Path:
    # Translate Unix-style '/c/' to Windows 'C:/' if needed
    new_path: Path
    if platform.system() == "Windows" and path.startswith("/c"):
        return Path("C:" + path[2:])
    else:
        return Path(path)

if __name__ == "__main__":
    strat_root = os.getenv("STRAT_ROOT")
    if strat_root is None:
        raise EnvironmentError("STRAT_ROOT environment variable must be set to the "
                                "absolute path to gen12_strategy/RaceSim")
    strat_root_path = resolve_path(strat_root)
    if not strat_root_path.exists():
        raise RuntimeError(f"STRAT_ROOT {strat_root_path} path doesn't exist")

    parser = argparse.ArgumentParser()
    parser.add_argument("--base_route", type=str, default="data/luts/fsgp/static/fsgp_base_route.csv",
                        help="Path to base route csv relative to STRAT_ROOT")
    args = parser.parse_args()

    base_route_path = strat_root_path / args.base_route
    App(base_route_path)
