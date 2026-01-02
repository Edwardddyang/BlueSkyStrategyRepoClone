"""
Interface with solcast API to retrieve live or historical forecast data
"""

import os
import requests
import json
import csv
import datetime
import argparse

from ratelimit import limits
import isodate
import geopy.distance

####################### Constants ##################################
# free trial
API_KEY_FREE = 'g7AehjAxhZPilJdO3aEyVRDmTikQAhs4'

# premium - Blue Sky's solcast account
API_KEY_PREMIUM = 'HwqyKe0MKq9VruZ4uTDTiejp4CF7Qm4a'

# (Kevin or Kenneth)'s API key
HISTORICAL_API_KEY = 'Pct5qaGHC6HN6kC8WGO-7pKtj1dx2kya'

# Historial API link
HISTORICAL_API = 'https://api.solcast.com.au/data/historic/radiation_and_weather'

# Forecast API link
FORECAST_API = 'https://api.solcast.com.au/data/forecast/radiation_and_weather'
####################################################################

def format_solcast_date(date: str) -> datetime:
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

def historic_call_by_site(lat: float, long: float,
                          start_time: datetime.datetime,
                          end_time: datetime.datetime,
                          output_parameters='dhi,dni,wind_speed_10m,wind_direction_10m',
                          output_file=None) -> json:
  '''
  Make an API call for a given site from the historical api

  Args:
  lat -- Latitude in degrees
  long -- Longitude in degrees
  start_time -- Period start
  end_time -- Period end
  output_parameters -- Solcast parameters to query for
  output_file -- Name of the output .json file to dump data

  Returns:
  JSON object with api response data
  '''
  if -90 > lat or lat > 90 or -180 > long or long > 180:
    raise ValueError(f"Latitude {lat} and longitude {long} values must be in the range of [-90, 90] "
                      "and [-180, 180] respectively.")

  start = start_time.astimezone(datetime.timezone.utc).isoformat()
  end = end_time.astimezone(datetime.timezone.utc).isoformat()
  payload = {
    'api_key': HISTORICAL_API_KEY,
    'latitude': lat, 
    'longitude':long,
    'output_parameters': output_parameters,
    'start': start,
    'end': end,
    'format': 'json',
  }

  headers = {
    'Content-Type': 'application/json'
  }

  call = requests.get(HISTORICAL_API, params = payload, headers = headers)

  if call.status_code != 200:
    raise RuntimeError(f"Historical site request failed with error code {call.status_code} and message {call.reason}")

  data = call.json()['estimated_actuals']

  if output_file is not None:
    if os.path.exists(os.path.dirname(output_file)) is None:
      raise RuntimeError(f"Output path for historical call {os.path.dirname} does not exist")

    with open(output_file, 'w') as file:
        json.dump(data, file, indent=4)

  return data


def forecast_call_by_site(lat: float, long: float,
                          output_parameters='dhi,dni,wind_speed_10m,wind_direction_10m',
                          paid=False, hours=168, output_file=None):
  """
  Make an API call for a given site from the (predictive) forecast API. All predictions are up to
  14 days into the future

  Args:
  lat -- Latitude in degrees
  long -- Longitude in degrees
  output_parameters -- Solcast parameters to query for
  paid -- Whether to use the subscription or free version of the API.
  hours -- The number of hours into the future to query for
  output_file -- Output .json file to dump data into
  """
  api_key: str
  if paid:
    api_key = API_KEY_PREMIUM
  else:
    api_key = API_KEY_FREE

  payload = {
    'api_key': api_key,
    'latitude': lat, 
    'longitude':long,
    'hours': hours,
    'output_parameters': output_parameters
  }
    
  headers = {
    'Content-Type': 'application/json'
  }

  call = requests.get(FORECAST_API, params = payload, headers = headers)
  if call.status_code != 200:
    raise RuntimeError(f"Forecast site request failed with error code {call.status_code} and message {call.reason}")

  data = call.json()['forecasts']

  if output_file is not None:
    if os.path.exists(os.path.dirname(output_file)) is None:
      raise RuntimeError(f"Output path for forecast call {os.path.dirname} does not exist")

    with open(output_file, 'w') as file:
        json.dump(data, file, indent=4)

  return data


def call_to_end(path: str, coords: list, index: int, call_type: str,
                start_time=None, end_time=None,
                irr_types='dhi,dni,wind_speed_10m,wind_direction_10m',
                paid = True, hours = 168):
  '''
  Find irradiance values for sites in a list starting from some index and writes these the data to a csv file
 
  Args:
  path (str): Path to output directory
  coords (list(tuple)): List of (latitude, longitude) pairs
  index (int): The index of the first site in coords to make forecasts for.
  call_type (string): Forecast ('forecast') or historical ('historical').
  start_time (datetime): The start timestamp in ISO 8601 format, timezone MUST be UTC.
                       Only required for historical call
  end_time (datetime): The end timestamp in ISO 8601 format. Timezone MUST be UTC.
                     Only required for historical call
  irr_types (list): Comma separated list of output parameters to query for
  paid (bool): If False, the free trial Solcast account is used. If True, the premium Solcast account is used.
               Default is True.
  hours (int): How many hours into the future to make forecasts for (NOTE: Max is 168). Default is 168
  
  Returns:
  nothing
  '''        

  if (index >= len(coords) or index < 0):
    raise RuntimeError(f"Index {index} is out of bounds of coords list")

  if call_type == 'historical' and (start_time is None or end_time is None):
    raise RuntimeError("Cannot call historical API without start and end times")

  if call_type == 'forecast' and hours == 0:
    raise RuntimeError("Cannot call forecast API with hours=0")
  
  if call_type == 'historical' and start_time == end_time:
    raise RuntimeError("Cannot call historical API with equal start and end times")

  if not os.path.exists(path) or not os.path.isdir(path):
    raise RuntimeError(f"Output directory {path} does not exist or is not a directory")

  # Sites to query for
  query_coords = coords[index:]

  # Hold data for each site. Note that at the end, len(calls) should equal len(query_coords)
  # This should be a list of lists when all calls are complete
  data = []

  print(f"Making {len(query_coords)} calls")
  for i in range(len(query_coords)):
    print(f"Call no. {i}")
    lat = query_coords[i][0]
    long = query_coords[i][1]
    if call_type == "historical":
      data.append(
        historic_call_by_site(lat, long, start_time, end_time, irr_types)
      )
    elif call_type == 'forecast':
      data.append(
        forecast_call_by_site(lat, long, irr_types, paid, hours)
      )
    else:
      raise RuntimeError(f"Unknown call type: {call_type}")

  # Hold timestamp data
  time_ls = []
  print("Finished making calls")
  for obj in data[0]:
    call_datetime = format_solcast_date(obj['period_end'])
    period_duration = obj['period']
    period_start = call_datetime - isodate.parse_duration(period_duration)
    datetime_int = period_start.strftime("%y%m%d%H%M%S")
    time_ls.append(datetime_int)

  # Create a csv for each output parameter queried for
  for irr_type in irr_types.split(","):
    parameter_csv_name = path + "/{}.csv".format(irr_type)
    with open(parameter_csv_name, "w", newline = '') as irr_csv:
      irr_csvWriter = csv.writer(irr_csv, delimiter = ',')
      irr_csvWriter.writerow(['latitude', 'longitude'] + time_ls)

  for i, site_data in enumerate(data):
    # Write data to its corresponding csv
    for irr_type in irr_types.split(","):
      # file name
      parameter_csv_name = path + "/{}.csv".format(irr_type)
      with open(parameter_csv_name, "a", newline = '') as irr_csv:
        irr_csvWriter = csv.writer(irr_csv, delimiter = ',')

        # Write data
        irr_ls = []
        for j in range(len(site_data)):
          irr_ls.append(site_data[j][irr_type])
        irr_csvWriter.writerow([query_coords[i][0],
                                query_coords[i][1]] + irr_ls)

def get_route_coordinates(route_file: str, starting_coord=None, interval=0.0):
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
        raise ValueError(f"Could not convert values to floats in row {i}: {row}")

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

  return coords

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--route', type=str, default=None, help='Path to route csv')
  parser.add_argument('--coord', type=float, default=None, nargs=2, metavar=('LAT', 'LONG'), help='Coordinate location')
  parser.add_argument('--interval', type=float, help="Number of km between coordinates")
  parser.add_argument('--call_type', type=str, default='forecast', choices=['historical', 'forecast'],
                      help="Type of solcast call. One of 'forecast' or 'historical'")
  parser.add_argument('--api_key', action="store_true", help="Use the subscription based API key or the free one")
  parser.add_argument('--hours', type=int, default=168, help="The number of hours to predict for forecast API")
  parser.add_argument('--starting_coord', type=float, nargs=2, metavar=('LAT', 'LONG'), help="Starting coordinate location. "
                      "Used with a route csv")
  parser.add_argument("--parameters", type=str, default='dhi,dni,wind_speed_10m,wind_direction_10m',
                      help="Comma separated list of parameters to query")
  parser.add_argument('--start_time', type=str, default=None,
                      help="Start time for historical API calls in YYYY-MM-DD HH:MM:SS format")
  parser.add_argument('--end_time', type=str, default=None,
                      help="End time for historical API calls in YYYY-MM-DD HH:MM:SS format")
  parser.add_argument('--utc_adjustment', type=float, default=None,
                      help="Offset from local time to UTC e.g. -5.5, +6.0")
  parser.add_argument('--output_csv', type=str, default=None, help="Path of the output csv")
  parser.add_argument('--output_json', type=str, default=None, help="Path to the output json file for single call by sites")
  args = parser.parse_args()

  start_time: datetime.datetime = None
  end_time: datetime.datetime = None
  utc_adjustment: datetime.timedelta = None

  if (args.start_time is None) != (args.end_time is None) or (args.start_time is None) != (args.utc_adjustment is None):
      raise RuntimeError("start_time, end_time, and utc_adjustment must all be defined or all be None.")

  if args.utc_adjustment is not None:
    utc_adjustment = datetime.timedelta(hours=args.utc_adjustment)

  if args.start_time is not None:
    try:
      start_time = datetime.datetime.strptime(args.start_time, "%Y-%m-%d %H:%M:%S").replace(tzinfo=datetime.timezone.utc)
      start_time = start_time + utc_adjustment
    except:
      raise RuntimeError(f"Start time {args.start_time} is not in YYYY-MM-DD HH:MM:SS format")
  
  if args.end_time is not None:
    try:
      end_time = datetime.datetime.strptime(args.end_time, "%Y-%m-%d %H:%M:%S").replace(tzinfo=datetime.timezone.utc)
      end_time = end_time + utc_adjustment
    except:
      raise RuntimeError(f"Start time {args.end_time} is not in YYYY-MM-DD HH:MM:SS format")

  # Make the appropriate call based on command line arguments
  if args.route is not None:
    coords = get_route_coordinates(args.route, args.starting_coord, args.interval)
    call_to_end(args.output_csv, coords, 0, args.call_type, start_time, end_time, args.parameters, args.api_key, args.hours)
  elif args.coord is not None:
    lat, long = args.coord
    if args.call_type == "historical":
      historic_call_by_site(lat, long, start_time, end_time, args.parameters, args.output_json)
    elif args.call_type == "forecast":
      forecast_call_by_site(lat, long, args.parameters, args.api_key, args.hours, args.output_json)
    else:
      raise RuntimeError(f"Unrecognized api {args.call_type}")
  else:
    raise RuntimeError("Pass in either a route csv or a coordinate")
