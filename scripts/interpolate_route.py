import csv
import math
import argparse

def haversine_distance(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000
    
    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Difference in coordinates
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Distance in meters
    distance = R * c
    
    return distance

def interpolate_coordinates(lat1, lon1, alt1, lat2, lon2, alt2, max_distance):
    # Calculate the number of intermediate points needed
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    num_points = int(distance // max_distance)
    
    # Generate intermediate points
    coordinates = []
    for i in range(1, num_points + 1):
      fraction = i / (num_points + 1)
      new_lat = lat1 + fraction * (lat2 - lat1)
      new_lon = lon1 + fraction * (lon2 - lon1)
      new_alt = alt1 + fraction * (alt2 - alt1)
      coordinates.append((new_lat, new_lon, new_alt))
    
    return coordinates

def interpolate_csv(input_file: str, output_file: str, max_distance: float, index_rows: bool):
  coords_list = []
  with open(input_file, 'r') as infile:
    reader = csv.reader(infile)
    for row in reader:
      if (len(row) < 3):
        raise RuntimeError("Need lat, lon, altitude sequence in each row of the csv")
      coords_list.append((float(row[0]), float(row[1]), float(row[2])))

      new_coords_list = []
      for i in range(len(coords_list) - 1):
        lat1, lon1, alt1 = coords_list[i]
        lat2, lon2, alt2 = coords_list[i + 1]
        new_coords_list.append((lat1, lon1, alt1))
        if haversine_distance(lat1, lon1, lat2, lon2) > max_distance:
          intermediate_coords = interpolate_coordinates(lat1, lon1, alt1, lat2, lon2, alt2, max_distance)
          new_coords_list.extend(intermediate_coords)
      new_coords_list.append(coords_list[-1])

    with open(output_file, 'w', newline='') as outfile:
      writer = csv.writer(outfile)
      count = 0
      for coords in new_coords_list:
        if index_rows:
          writer.writerow([count, coords[0], coords[1], coords[2]])
          count += 1
        else:
          writer.writerow([coords[0], coords[1], coords[2]])

if __name__ == "__main__":
  parser = argparse.ArgumentParser(
    description="Interpolate route points such that distances between all consecutive points lie below a threshold"
  )
  parser.add_argument("input_csv", type=str, help="Input route csv with columns |latitude|longitude|altitude|")
  parser.add_argument("--threshold", type=float, default=10.0, help="Maximum distance between consecutive points")
  parser.add_argument("--output_csv", type=str, default="out.csv", help="Name of output csv")
  parser.add_argument("--index_rows", action="store_true", help="Attach an index column")
  args = parser.parse_args()
  
  interpolate_csv(args.input_csv, args.output_csv, args.threshold, args.index_rows)
