import pandas as pd
import plotly.graph_objects as go
import numpy as np

#Load Base Route CSV
csv_path = "../data/luts/fsgp/static/fsgp_base_route.csv"
df = pd.read_csv(csv_path, header=None, names=["lat", "lon", "alt"])

lat = df["lat"].values
lon = df["lon"].values
points = np.column_stack((lon, lat))

#Load Results CSV
csv_path = "../Acceleration_Results/race_plan.csv"
df = pd.read_csv(csv_path)

loop = df["Loop"].values

df["Segment"] = df["Segment"].apply(lambda s: [int(x) for x in s.split()])
segment = df["Segment"].values

df["Speed"] = df["Speed"].apply(lambda s: [float(x) for x in s.split()])
speed = df["Speed"].values

acceleration = df["Acceleration"].values
points2 = np.column_stack((loop, segment, speed, acceleration))

# Get to the correct loop
loop = 30
route_idx = 0
while route_idx < len(points2) and points2[route_idx][0] != loop:
    route_idx += 1

# Show segments for the route
segments = []

for i in range(len(points) - 1):
    if i == points2[route_idx][1][1]:
        route_idx += 1
    x = [points[i][0], points[i+1][0]]
    y = [points[i][1], points[i+1][1]]
    
    line_color = 'black' 
    if points2[route_idx][3] < 0.0:
        line_color = 'red'
    elif points2[route_idx][3] > 0.0:
        line_color = 'green'

    segments.append(go.Scatter(
        x=x, y=y, mode='lines',
        line=dict(color=line_color, width=4),
        hoverinfo='text',
        text=f"""Segment {points2[route_idx][1][0]} to {points2[route_idx][1][1]}
Speed:{points2[route_idx][2][0]:.2f} to {points2[route_idx][2][1]:.2f} m/s
Acceleration: {points2[route_idx][3]:.2f} m/s²"""
    ))

# Show figure
fig = go.Figure(data=segments)


    
fig.update_layout(
    title=f"FSGP Track Route Loop: {loop}",
    xaxis_title="Longitude",
    yaxis_title="Latitude",
    showlegend=False,
    height=700,
)

fig.show()
