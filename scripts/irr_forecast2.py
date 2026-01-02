"""
Interface with openmeteo API to retrieve live or historical forecast data
"""

import os
import requests
import json
import csv
import datetime
from datetime import timedelta

import argparse

from typing import Optional
from ratelimit import limits
from pathlib import Path
import isodate
import geopy.distance

####################### Constants ##################################

# DEFAULT Open Meteo API link
FORECAST_API_URL_TEMPLATE = (
  "https://api.open-meteo.com/v1/forecast?"
  "latitude={lat}&longitude={long}"
  "&hourly={output_parameters}&forecast_days={days}"
)

# DEFAULT Open Meteo Historical API link
HISTORICAL_API_URL_TEMPLATE = (
  "https://archive-api.open-meteo.com/v1/archive?"
  "latitude={lat}&longitude={long}&start_date={start_date}"
  "&end_date={end_date}&hourly={output_parameters}"
)

####################################################################

def format_date(date: str) -> datetime: 
  """
  Utility function to convert iso 8601 timestamp to python datetime object

  Args:
  date -- ISO 8601 timestamp as a string

  Returns:
  datetime object representing the input timestamp
  """
  try:
    datetime_obj = datetime.datetime.fromisoformat(date)
  except:
    raise RuntimeError(f"Could not convert time ISO 8601 timestamp {date} to datetime")
      
  return datetime_obj


def historic_call_by_site(lat: float=37.001051, long: float=-86.368572,  # Start line of FSGP in National Corvette Museum. Kentucky, US
                          start_time: datetime.datetime = datetime.datetime.fromisoformat('2025-01-01'),
                          end_time: datetime.datetime = datetime.datetime.fromisoformat('2025-05-15'),
                          output_parameters: str='wind_speed_10m,wind_direction_10m,shortwave_radiation',
                          dump_dir: Optional[Path]=None) -> json:
  '''
  Make an API call for a given site from the historical api

  Args:
  lat -- Latitude in degrees
  long -- Longitude in degrees
  start_time -- Period start
  end_time -- Period end
  output_parameters -- Solcast parameters to query for
  dump_dir -- Dump directory for the output json

  Returns:
  JSON object with api response data
  '''
  if -90 > lat or lat > 90 or -180 > long or long > 180:
    raise ValueError(f"Latitude {lat} and longitude {long} values must be in the range of [-90, 90] "
                      "and [-180, 180] respectively.")

  historical_api = HISTORICAL_API_URL_TEMPLATE.format(
    lat=lat, long=long, start_date=start_time.strftime("%Y-%m-%d"), end_time=end_time.strftime("%Y-%m-%d"), output_parameters=output_parameters
  )
  
  call = requests.get(historical_api)

  if call.status_code != 200:
    raise RuntimeError(f"Historical site request failed with error code {call.status_code} and message {call.reason}")

  data = call.json()

  if dump_dir is not None:
    if not dump_dir.exists():
      raise RuntimeError(f"Output json dump directory for historical call {dump_dir} does not exist")

    with open(dump_dir / "historic.json", 'w') as file:
      json.dump(data, file, indent=4)

  return data



def forecast_call_by_site(lat: float = 37.001051, long: float = -86.368572, days: int = 7,
                          output_parameters: str = 'wind_speed_10m,wind_direction_10m,shortwave_radiation',
                          dump_dir: Optional[Path] = None):
  """
  Make an API call for a given site from the (predictive) forecast API. All predictions are up to
  14 days into the future

  Args:
  lat -- Latitude in degrees
  long -- Longitude in degrees
  days -- The number of days into the future to query for. Must be 1, 3, 7, 14, or 16
  output_parameters -- Solcast parameters to query for
  dump_dir -- Dump directory for the output json
  """
  if -90 > lat or lat > 90 or -180 > long or long > 180:
    raise ValueError(f"Latitude {lat} and longitude {long} values must be in the range of [-90, 90] "
                      "and [-180, 180] respectively.")
  
  if days not in [1, 3, 7, 14, 16]:
    raise ValueError(f"Days {days} must be one of [1, 3, 7, 14, 16]")

  FORECAST_API = f'https://api.open-meteo.com/v1/forecast?latitude={lat}&longitude={long}&hourly={output_parameters}&forecast_days={days}'
  forecast_api = FORECAST_API_URL_TEMPLATE.format(
    lat=lat, long=long, output_parameters=output_parameters, days=days
  )

  call = requests.get(forecast_api)
  if call.status_code != 200:
    raise RuntimeError(f"Forecast site request failed with error code {call.status_code} and message {call.reason}")

  data = call.json()
  if dump_dir is not None:
    if not dump_dir.exists():
      raise RuntimeError(f"Output directory for forecast call {output_file} does not exist")

    with open(dump_dir / "forecast.json", 'w') as file:
      json.dump(data, file, indent=4)

  return data


def call_to_end(call_type: str,
                coords: list,
                dump_dir: Path,
                start_time: Optional[datetime.datetime] = None,
                end_time : Optional[datetime.datetime] = None,
                days: int = 7,
                utc_adjustment: int=6,
                irr_types: str = 'wind_speed_10m,wind_direction_10m,shortwave_radiation'):
  '''
  Find irradiance values for lat/long sites in a list starting and write the data to a csv file
 
  Args:
  call_type (string): Forecast ('forecast') or historical ('historical').
  coords (list): List of lat/long tuples for which to make calls
  dump_dir (Path): Path to output directory for dumping all result .csv, .json etc.
  start_time (datetime): The start timestamp in ISO 8601 format, timezone MUST be UTC.
                       Only required for historical call
  end_time (datetime): The end timestamp in ISO 8601 format. Timezone MUST be UTC.
                     Only required for historical call
  days (int) -- The number of days into the future to query for. Must be 1, 3, 7, 14, or 16
  irr_types (string): Comma separated list of output parameters to query for
  
  Returns:
  nothing
  '''
  if call_type == 'historical' and (start_time is None or end_time is None):
    raise RuntimeError("Cannot call historical API without start and end times")

  if call_type == 'forecast' and days == 0:
    raise RuntimeError("Cannot call forecast API with days=0")
  
  if call_type == 'historical' and start_time == end_time:
    raise RuntimeError("Cannot call historical API with equal start and end times")

  if len(coords) <= 0:
    raise ValueError("Coordinates list must have at least one value")

  if days not in {1, 3, 7, 14, 16}:
    raise ValueError("Days is not one of {1, 3, 7, 14, 16}")

  # Holds data
  data = []
  
  print(f"Making {len(coords)} calls")
  for i in range(len(coords)):
    print(f"Call no. {i}")
    lat = coords[i][0]
    long = coords[i][1]
    if call_type == "historical":
      data.append(
        historic_call_by_site(lat, long, start_time, end_time, irr_types)
      )
    elif call_type == 'forecast':
      data.append(
        forecast_call_by_site(lat, long, days, irr_types)
      )
    else:
      raise RuntimeError(f"Unknown call type: {call_type}")

  # Hold timestamp data
  time_ls = []
  print("Finished making calls")
  for time in data[0]['hourly']['time']:
    call_datetime = format_date(time) + timedelta(hours=6)
    datetime_int = call_datetime.strftime("%Y-%m-%dT%H:%M:%S") 
    time_ls.append(datetime_int)
    
  # Create a csv for each output parameter queried for
  for irr_type in irr_types.split(","):
    parameter_csv_name = dump_dir / "{}.csv".format(irr_type)
    with open(parameter_csv_name, "w", newline = '') as irr_csv:
      irr_csvWriter = csv.writer(irr_csv, delimiter = ',')
      irr_csvWriter.writerow(['latitude', 'longitude'] + time_ls)

  for i, site in enumerate(data):
    for site_data in enumerate(site['hourly']):
      if site_data == 'time':
        continue

      # Write data to its corresponding csv
      # file name
      parameter_csv_name = dump_dir / "{}.csv".format(site_data[1])
      with open(parameter_csv_name, "a", newline = '') as irr_csv:
        irr_csvWriter = csv.writer(irr_csv, delimiter = ',')    
          
        # Write data
        irr = site['hourly'][site_data[1]]
        irr_csvWriter.writerow([coords[i][0], coords[i][1]] + irr)

def get_route_coordinates(route_file: str, starting_coord=None, interval=0.0) -> list:
  '''
  Return the array of route coordinates from a csv

  Args:
  route_file -- Path to the |lat|lon|... .csv file with all the coordinates of the route
  starting_coord -- Starting coordinate (lat, long) tuple
  interval -- number of km between coordinates

  Return:
  List of coordinate tuples
  '''
  # Hold all coordinates from the input csv
  coords_tuple = []

  if not os.path.isfile(route_file):
    raise FileNotFoundError(f"The file {route_file} does not exist.")
  if interval < 0.0:
    raise ValueError(f"Interval {interval} must be >= 0")

  # Collect all route coordinates
  with open(route_file, 'r') as file:
    reader = csv.reader(file)
    for i, row in enumerate(reader):
      if len(row) < 2:
        raise RuntimeError(f"Row {i} does not have at least 2 elements")
      try:
        lat = float(row[0])
        lon = float(row[1])
        coords_tuple.append((lat, lon))
      except:
        raise ValueError(f"Could not convert lat/long values to floats in row {i}: {row}")

  # Find the starting index
  starting_index = 0
  if starting_coord is not None:
    max_distance = float("inf")
    for i, coord in enumerate(coords_tuple):
      dist = geopy.distance.distance(coord, starting_coord).km
      if dist < max_distance:
        starting_index = i
        max_distance = dist

    coords_tuple = coords_tuple[starting_index:]

  # Remove coordinates that are within interval km from each other
  coords = []
  next_lat_lon = coords_tuple[0]
  coords.append(coords_tuple[0])
  for lat_lon in coords_tuple:
    if geopy.distance.distance(lat_lon, next_lat_lon).km > interval:
      next_lat_lon = lat_lon
      coords.append(next_lat_lon)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--route', type=str, default=None, help='Path to a route csv with a list of |latitude|longitude| coordinates')
  parser.add_argument('--coord', type=float, default=None, nargs=2, metavar=('LAT', 'LONG'), help='Coordinate location for a single call')
  parser.add_argument('--interval', type=float, help="Number of km between coordinates")
  parser.add_argument('--call_type', type=str, default='forecast', choices=['historical', 'forecast'],
                      help="Type of API call. One of 'forecast' or 'historical. Default to forecast'")
  parser.add_argument('--days', type=int, choices={1, 3, 7, 14, 16}, default=7, help="The number of days to predict if the forecast API was chosen."
                                                                                      "Must be one of {1, 3, 7, 14, 16}")
  parser.add_argument('--starting_coord', type=float, nargs=2, metavar=('LAT', 'LONG'), help="Starting coordinate location. "
                     "Used if a route csv was passed in")
  parser.add_argument("--parameters", type=str, default='wind_speed_10m,wind_direction_10m,shortwave_radiation',
                      help="Comma separated list of parameters to query. See open-meteo documentation for query-able parameters")
  parser.add_argument('--dump_dir', type=str, default=".", help="Path to the directory in which to dump the output csv, json. Default is pwd")
  parser.add_argument('--start_time', type=str, default=None,
                      help="Start time in YYYY-MM-DD HH:MM:SS format if the historical API call was chosen")
  parser.add_argument('--end_time', type=str, default=None,
                      help="Start time in YYYY-MM-DD HH:MM:SS format if the historical API call was chosen")
  parser.add_argument('--utc_adjustment', type=float, default=None,
                      help="Offset from local time to UTC e.g. -5.5, +6.0")

  args = parser.parse_args()

  start_time: datetime.datetime = None
  end_time: datetime.datetime = None
  utc_adjustment: datetime.timedelta = None
  dump_dir = Path(args.dump_dir)

  if (args.start_time is None) != (args.end_time is None) or (args.start_time is None) != (args.utc_adjustment is None):
    raise RuntimeError("start_time, end_time, and utc_adjustment must all be defined or all be None.")

  if args.utc_adjustment is not None:
    utc_adjustment = datetime.timedelta(hours=args.utc_adjustment)

  if args.start_time is not None:
    try:
      # Convert to UTC by applying user defined adjustment
      start_time = datetime.datetime.strptime(args.start_time, "%Y-%m-%d %H:%M:%S").replace(tzinfo=datetime.timezone.utc)
      start_time = start_time + utc_adjustment
    except:
      raise RuntimeError(f"Start time {args.start_time} is not in YYYY-MM-DD HH:MM:SS format")
  
  if args.end_time is not None:
    try:
      # Convert to UTC by applying user defined adjustment
      end_time = datetime.datetime.strptime(args.end_time, "%Y-%m-%d %H:%M:%S").replace(tzinfo=datetime.timezone.utc)
      end_time = end_time + utc_adjustment
    except:
      raise RuntimeError(f"Start time {args.end_time} is not in YYYY-MM-DD HH:MM:SS format")

  # Make the appropriate call based on command line arguments
  if args.route is not None:
    coords = get_route_coordinates(args.route, args.starting_coord, args.interval)
    call_to_end(args.call_type, coords, dump_dir, start_time, end_time, args.days, args.parameters)
  elif args.coord is not None:
    call_to_end(args.call_type, [args.coord], dump_dir, start_time, end_time, args.days,
                args.parameters)
  else:
    raise RuntimeError("Pass in either a route csv or a coordinate")      

