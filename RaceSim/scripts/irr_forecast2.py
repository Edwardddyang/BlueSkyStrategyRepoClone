"""
Interface with openmeteo API to retrieve live or historical forecast data
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

# DEFAULT Open Meteo API link
FORECAST_API = 'https://api.open-meteo.com/v1/forecast?latitude=37.001051&longitude=-86.368572&hourly=wind_speed_10m,shortwave_radiation,wind_direction_10m&timezone=auto&forecast_days=7'

# DEFAULT Open Meteo Historical API link
HISTORICAL_API = 'https://archive-api.open-meteo.com/v1/archive?latitude=37.001051&longitude=-86.368572&start_date=2025-01-01&end_date=2025-05-15&hourly=wind_speed_10m,wind_direction_10m,shortwave_radiation'

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


def historic_call_by_site(lat: float=37.001051, long: float=-86.368572,
                          start_time: datetime.datetime = datetime.datetime.fromisoformat('2025-01-01'),
                          end_time: datetime.datetime = datetime.datetime.fromisoformat('2025-05-15'),
                          output_parameters='wind_speed_10m,wind_direction_10m,shortwave_radiation',
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
  
  HISTORICAL_API = f'https://archive-api.open-meteo.com/v1/archive?latitude={lat}&longitude={long}&start_date={start_time.strftime("%Y-%m-%d")}&end_date={end_time.strftime("%Y-%m-%d")}&hourly={output_parameters}'
  
  call = requests.get(HISTORICAL_API)

  if call.status_code != 200:
    raise RuntimeError(f"Historical site request failed with error code {call.status_code} and message {call.reason}")

  data = call.json()

  if output_file is not None:
    if os.path.exists(os.path.dirname(output_file)) is None:
      raise RuntimeError(f"Output path for historical call {os.path.dirname} does not exist")

    with open(output_file, 'w') as file:
        json.dump(data, file, indent=4)

  return data



def forecast_call_by_site(lat: float=37.001051, long: float=-86.368572,days=7,
                          output_parameters='wind_speed_10m,wind_direction_10m,shortwave_radiation',
                          output_file=None):
  """
  Make an API call for a given site from the (predictive) forecast API. All predictions are up to
  14 days into the future

  Args:
  lat -- Latitude in degrees
  long -- Longitude in degrees
  days -- The number of days into the future to query for. Must be 1, 3, 7, 14, or 16
  output_parameters -- Solcast parameters to query for
  output_file -- Output .json file to dump data into
  """
  if -90 > lat or lat > 90 or -180 > long or long > 180:
    raise ValueError(f"Latitude {lat} and longitude {long} values must be in the range of [-90, 90] "
                      "and [-180, 180] respectively.")
  
  if days not in [1, 3, 7, 14, 16]:
    raise ValueError(f"Days {days} must be one of [1, 3, 7, 14, 16]")

  FORECAST_API = f'https://api.open-meteo.com/v1/forecast?latitude={lat}&longitude={long}&hourly={output_parameters}&forecast_days={days}'

  call = requests.get(FORECAST_API)
  if call.status_code != 200:
    raise RuntimeError(f"Forecast site request failed with error code {call.status_code} and message {call.reason}")

  data = call.json()

  if output_file is not None:
    if os.path.exists(os.path.dirname(output_file)) is None:
      raise RuntimeError(f"Output path for forecast call {os.path.dirname} does not exist")

    with open(output_file, 'w') as file:
        json.dump(data, file, indent=4)

  return data


def call_to_end(call_type,
                path: str = '../data/luts/fsgp/dynamic',
                start_time=None, end_time=None,days = 7,
                irr_types='wind_speed_10m,wind_direction_10m,shortwave_radiation'):
  '''
  Find irradiance values for sites in a list starting from some index and writes these the data to a csv file
 
  Args:
  call_type (string): Forecast ('forecast') or historical ('historical').
  path (str): Path to output directory
  start_time (datetime): The start timestamp in ISO 8601 format, timezone MUST be UTC.
                       Only required for historical call
  end_time (datetime): The end timestamp in ISO 8601 format. Timezone MUST be UTC.
                     Only required for historical call
  days -- The number of days into the future to query for. Must be 1, 3, 7, 14, or 16
  irr_types (list): Comma separated list of output parameters to query for
  
  Returns:
  nothing
  '''        

  

  if call_type == 'historical' and (start_time is None or end_time is None):
    raise RuntimeError("Cannot call historical API without start and end times")

  if call_type == 'forecast' and days == 0:
    raise RuntimeError("Cannot call forecast API with days=0")
  
  if call_type == 'historical' and start_time == end_time:
    raise RuntimeError("Cannot call historical API with equal start and end times")

 

  if not os.path.exists(path) or not os.path.isdir(path):
    raise RuntimeError(f"Output directory {path} does not exist or is not a directory")

  
  #Holds data
  data = {}

  
  if call_type == "historical":
    data = historic_call_by_site(start_time=start_time, end_time=end_time, output_parameters=irr_types)
    
  elif call_type == 'forecast':
    data = forecast_call_by_site(days=days, output_parameters=irr_types)
  else:
    raise RuntimeError(f"Unknown call type: {call_type}")
  
  #Hold timestamp data
  time_ls = []
  print("Finished making calls")
  for time in data['hourly']['time']:
    call_datetime = format_date(time)
    datetime_int = call_datetime.strftime("%y-%m-%d %H:%M:%S")
    time_ls.append(datetime_int)
    
  # Create a csv for each output parameter queried for
  for irr_type in irr_types.split(","):
    parameter_csv_name = path + "/{}.csv".format(irr_type)
    with open(parameter_csv_name, "w", newline = '') as irr_csv:
      irr_csvWriter = csv.writer(irr_csv, delimiter = ',')
      irr_csvWriter.writerow(time_ls)
      
  for site_data in enumerate(data['hourly']):
    if site_data == 'time':
      continue

    # Write data to its corresponding csv
    # file name
    parameter_csv_name = path + "/{}.csv".format(site_data[1])
    with open(parameter_csv_name, "a", newline = '') as irr_csv:
      irr_csvWriter = csv.writer(irr_csv, delimiter = ',')    
        
      # Write data
      irr = data['hourly'][site_data[1]]
      irr_csvWriter.writerow(irr)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  #parser.add_argument('--route', type=str, default=None, help='Path to route csv')
  parser.add_argument('--coord', type=float, default=None, nargs=2, metavar=('LAT', 'LONG'), help='Coordinate location')
  parser.add_argument('--interval', type=float, help="Number of km between coordinates")
  parser.add_argument('--call_type', type=str, default='forecast', choices=['historical', 'forecast'],
                      help="Type of solcast call. One of 'forecast' or 'historical'")
  #parser.add_argument('--api_key', action="store_true", help="Use the subscription based API key or the free one")
  parser.add_argument('--days', type=int, default=7, help="The number of days to predict for forecast API. Must be 1, 3, 7, 14, or 16")
  #parser.add_argument('--starting_coord', type=float, nargs=2, metavar=('LAT', 'LONG'), help="Starting coordinate location. "
  #                    "Used with a route csv"
  parser.add_argument("--parameters", type=str, default='wind_speed_10m,wind_direction_10m, shortwave_radiation',
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
  #if args.route is not None:
    #coords = get_route_coordinates(args.route, args.starting_coord, args.interval)
    
  call_to_end(args.call_type, args.output_csv, start_time, end_time, args.days, args.parameters)
  if args.coord is not None:
    lat, long = args.coord
    if args.call_type == "historical":
      historic_call_by_site(lat, long, start_time, end_time, args.parameters, args.output_json)
    elif args.call_type == "forecast":
      forecast_call_by_site(lat, long, args.days, args.parameters, args.output_json)
    else:
      raise RuntimeError(f"Unrecognized api {args.call_type}")
  else:
    raise RuntimeError("Pass in either a route csv or a coordinate")      

