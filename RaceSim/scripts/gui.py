from dash import Dash, html, dcc, Output, Input, callback
import pandas as pd
import plotly.graph_objects as go
import json
import numpy as np
import os

# Load Base Route CSV
route_csv = "../data/luts/fsgp/static/fsgp_base_route.csv"
df = pd.read_csv(route_csv, header=None, names=["lat", "lon", "alt"])

lat = df["lat"].values
lon = df["lon"].values
points = np.column_stack((lon, lat))

# Load Race Plan JSON
race_plan_folder = "../exports"

race_files = [f for f in os.listdir("../exports") if f.endswith(".json")]


app = Dash()

# Dsah Layout
app.layout = html.Div(
    [
        # Stores the JSON data
        dcc.Store(id="json"),
        html.Div(
            children=[
                # Choose JSON file
                html.Label("JSON File:"),
                dcc.Dropdown(
                    id="files",
                    options=race_files,
                    value=race_files[0],
                    style={"width": "300px", "marginRight": "20px"},
                ),
                # Choose Loop
                html.Label("Loop:"),
                dcc.Input(
                    id="loop",
                    value="1",
                    type="number",
                    style={"width": "60px"},
                ),
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


# Loading JSON data
@callback(Output("json", "data"), Input("files", "value"))
def load_json(selected_json):
    # Load the selected JSON file
    file_path = race_plan_folder + "/" + selected_json
    with open(file_path, "r") as file:
        race_data = json.load(file)

    all_segments = []

    # For each loop
    for loop in race_data:
        route_idx = 0
        loop_segments = []

        # For each segment in the loop
        for seg in range(len(points) - 1):
            if route_idx >= len(loop):
                break

            x_coords = [points[seg][0], points[seg + 1][0]]
            y_coords = [points[seg][1], points[seg + 1][1]]
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
Speed: {speed_start:.2f} to {speed_end:.2f} m/s<br>
Acceleration: {accel:.2f} m/s²""",
                }
            )

        all_segments.append(loop_segments)
    return all_segments


@callback(Output("graph", "figure"), Input("loop", "value"), Input("json", "data"))
def update_graph(loop, all_segments):

    # Check if loop is valid
    try:
        loop_index = int(loop) - 1
        if loop_index < 0 or loop_index >= len(all_segments):
            raise IndexError
    except (ValueError, TypeError, IndexError):
        # Default value (empty or invalid loop)
        return {
            "data": [],
            "layout": {
                "title": "Invalid or Empty loop input",
                "xaxis": {"title": "Longitude"},
                "yaxis": {"title": "Latitude"},
                "height": 700,
            },
        }
    # Valid loop
    return {
        "data": all_segments[loop_index],
        "layout": {
            "xaxis": {"title": "Longitude"},
            "yaxis": {"title": "Latitude"},
            "showlegend": False,
            "height": 700,
        },
    }


if __name__ == "__main__":
    app.run(debug=True)
