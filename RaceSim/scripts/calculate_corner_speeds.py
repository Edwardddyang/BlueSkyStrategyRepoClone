import numpy as np
import csv
from scipy.optimize import least_squares
import argparse
import sys

R = 6371000  # Earth radius in meters

def latlon_to_cartesian(lat, lon):
  lat = np.radians(lat)
  lon = np.radians(lon)
  x = R * np.cos(lat) * np.cos(lon)
  y = R * np.cos(lat) * np.sin(lon)
  z = R * np.sin(lat)
  return x, y, z

def haversine(lat1, lon1, lat2, lon2):
  """Calculate distance in m between two lat, lon coordinates"""
  phi1, phi2 = np.radians(lat1), np.radians(lat2)
  dphi = np.radians(lat2 - lat1)
  dlambda = np.radians(lon2 - lon1)
  a = np.sin(dphi / 2) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2) ** 2
  return 2 * R * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

def calc_radius(params, x, y, z):
  a, b, c, r = params
  return np.sqrt((x - a)**2 + (y - b)**2 + (z - c)**2) - r

def get_coords(csv_path: str) -> list:
  """Load a csv of |Latitude|Longitude|Altitude| columns"""
  coords = []
  with open(csv_path, 'r') as file:
    reader = csv.reader(file)

    count = 0
    for row in reader:
      if (len(row) < 3):
        raise RuntimeError(f"Row {str(count+1)} must have values for latitude, longitude and altitude")
      coords.append((float(row[0]), float(row[1]), float(row[2])))
      count += 1

  return coords

def get_corners(csv_path: str) -> list:
  """Load a csv of |corner start index|corner end index| columns"""
  corners = []
  with open(csv_path, 'r') as file:
    reader = csv.reader(file)

    count = 0
    for row in reader:
      if (len(row) < 2):
        raise RuntimeError(f"Row {str(count+1)} must have values for the corner start index and the corner end index")
      corners.append((int(row[0]), int(row[1])))
      count += 1

  return corners

def fit_circle(x, y):
  def calc_R(xc, yc):
    """Calculate radius"""
    return np.sqrt((x - xc) ** 2 + (y - yc) ** 2)

  def f(c):
    Ri = calc_R(*c)
    return Ri - Ri.mean()

  center_estimate = np.mean(x), np.mean(y)
  center= least_squares(f, center_estimate).x
  radius = calc_R(*center).mean()
  return center, radius

def calculate_corner(coords: list, corners: list) -> list:
  """Calculate corner radii and centers given a list of coordinates and the corner segments"""
  num_coords = len(coords)
  radii = []
  circle_centers = []
  for corner in corners:
    corner_start = corner[0]
    corner_end = corner[1]

    if (corner_start < 0 or corner_start >= num_coords):
      raise RuntimeError(f"Corner start {corner_start} falls out of bounds")
  
    if (corner_end < 0 or corner_end >= num_coords):
      raise RuntimeError(f"Corner end {corner_end} falls out of bounds")
    
    if (corner_start > corner_end):
      raise RuntimeError(f"Corner start {corner_start} must come before {corner_end}")

    x_coords = [coord[0] for coord in coords[corner_start:corner_end+1]]
    y_coords = [coord[1] for coord in coords[corner_start:corner_end+1]]
    center, radius = fit_circle(np.array(x_coords), np.array(y_coords))

    radii.append(radius)
    circle_centers.append(center)
  
  return radii, circle_centers

if __name__ == "__main__":
  parser = argparse.ArgumentParser(
    description="Calculate corner speeds given a csv of coordinates and a csv of corner intervals"
  )
  parser.add_argument("coord_csv", type=str, help="Input route csv with columns |latitude|longitude|altitude|")
  parser.add_argument("corners_csv", type=str, help="Input corner csv with columns |corner start index|corner end index|")
  args = parser.parse_args()
  
  coords = get_coords(args.coord_csv)
  corners = get_corners(args.corners_csv)

  cartesian_coords = np.array([latlon_to_cartesian(coord[0], coord[1]) for coord in coords])
  radii, centers = calculate_corner(cartesian_coords, corners)
  
  print(radii, len(radii))
