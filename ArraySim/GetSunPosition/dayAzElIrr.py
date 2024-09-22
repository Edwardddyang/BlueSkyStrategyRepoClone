"""
Generate a 180 x 4 csv with columns: 
azimuth (degrees) | elevation (degrees) | irradiance (W/m^2) | local time (24 hour)
"""

from pysolar.solar import *
import datetime
import csv
import requests
import json
from pathlib import Path
from argparse import ArgumentParser

KEVIN_API_KEY = 'EYiCk5D1-LsSyduwPHGc_QyB4pZzeBfp' # Kevin's API key
KENNETH_API_KEY = 'Pct5qaGHC6HN6kC8WGO-7pKtj1dx2kya' # Kenneth's API key
API_KEY = KENNETH_API_KEY
API_URL = 'https://api.solcast.com.au/data/historic/radiation_and_weather'
RACE_START_TIME = datetime.datetime.strptime("2023-10-13 08:59:00", '%Y-%m-%d %H:%M:%S') # Only the HH:MM:SS fields are used
RACE_END_TIME = datetime.datetime.strptime("2023-10-13 17:00:00", '%Y-%m-%d %H:%M:%S') # Only the HH:MM:SS fields are used
SOLCAST_KEYS_METADATA_PATH = Path(__file__).resolve().parent / "solcast_keys.json"

def format_solcast_date(date: str):
    '''Converts a Solcast timestamp to a python datetime object'''
    format = '%Y-%m-%dT%H:%M:%S%z'
    datetime_obj = datetime.datetime.strptime(date, format).replace(tzinfo=datetime.timezone.utc)
    return (datetime_obj)

def compare_clock_time(date1: datetime.datetime, date2: datetime.datetime):
    """
    Compare two datetimes by their HH:MM:SS only
    Returns true if date1 is greater or equal to date2, false otherwise
    """
    if (date1.hour > date2.hour):
        return True
    elif (date1.hour < date2.hour):
        return False
    
    if (date1.minute > date2.minute):
        return True
    elif (date1.minute < date2.minute):
        return False
    
    if (date1.second > date2.second):
        return True
    elif (date1.second < date2.second):
        return False
    
    return True

def historic_call_by_site(lat: float, long: float, start: str, end: str, output_parameters: str="dni,dhi"):
    '''
    Makes an API call for a given site from the historical api
    @param lat: latitude in degrees
    @param long: longitude in degrees
    @param output_parameters: comma seperated list of parameters to get from the api. Refer
    to the documentation for a specific list of supported parameters
    @param start: Start time in ISO_8601 format UTC
    @param end: End time in ISO_8601 format UTC
    
    Return: Json object with API return data
    '''

    # Ensure that start time is before end time
    start_time = datetime.datetime.fromisoformat(start)
    end_time = datetime.datetime.fromisoformat(end)

    if start_time >= end_time:
        raise RuntimeError(f"Start time {start_time} must be before {end_time}")

    payload = {
        'api_key': API_KEY,
        'latitude': lat, 
        'longitude':long,
        'output_parameters': output_parameters,
        'start': start,
        'end': end,
        'period': 'PT5M',
        'format': 'json',
    }

    headers = {
        'Content-Type': 'application/json'
    }

    call = requests.get(API_URL, params = payload, headers = headers)
    data = call.json()['estimated_actuals']

    return data

def get_irradiance(data: list, curr_time_utc: datetime.datetime, index_cache: int):
    """
    Get irradiance value according to the closest time to curr_time_utc starting at data[index_cache]
    @param data: list of return values from solcast 
    example: [{'dhi': 20, 'dni': 115, 'period_end': '2023-10-21T21:00:00.0000000Z', 'period': 'PT30M'}, {'dhi': 54, 'dni': 407, 'period_end': '2023-10-21T21:30:00.0000000Z', 'period': 'PT30M'}]
    @param curr_time_utc: time to retrieve data for
    @param index_cache: the index in data at which to begin searching for the closest data point
    Returns: The relevant data point and the new index cache
    """
    data_time = format_solcast_date(data[index_cache]['period_end'])
    # Still within the same interval
    if (curr_time_utc < data_time or index_cache >= len(data)):
        pass
    # In next interval
    else:
        index_cache += 1
        
    return data[index_cache], index_cache


def generate_irr_csv(lat: float, lon: float, start_time: str, end_time: str, utc_adjustment: str, num_timesteps: int, output_csv_name: str):
    """
    Generates the 180 x 4 csv and saves it to a sun_position.csv file in the current directory
    @param lat: latitude in degrees
    @param lon: longitude in degrees
    @param start_time: start time of the simulation in "YYYY-MM-DD HH:MM:SS" format in 24 hour local time
    @param end_time: end time of the simulation in "YYYY-MM-DD HH:MM:SS" format in 24 hour local time
    @param utc_adjustment: conversion from local time to utc. Note that for Alice Springs Australia, this is -09:30:00
    @param num_timesteps: Number of timesteps between start_time and end_time inclusive

    Example: python dayAzElIrr.py -23.6980 133.8807 "2023-10-22 06:00:00" "2023-10-22 19:00:00" -9.5 180 180 to simulate in Alice Springs
    with a bearing of 180 degrees (pointing south) and 180 timesteps
    """
    # Local Times
    sim_start_time = datetime.datetime.strptime(start_time, '%Y-%m-%d %H:%M:%S')
    sim_end_time = datetime.datetime.strptime(end_time, '%Y-%m-%d %H:%M:%S')

    # UTC Times
    sign = utc_adjustment[0]
    if sign != "+" and sign != "-":
        raise RuntimeError("UTC Adjustment must start with a + or - indicating the direction of adjustment from local time to UTC")
    hours, minutes, seconds = map(int, utc_adjustment[1:].split(':'))

    # Calculate the total seconds
    total_seconds = hours * 3600 + minutes * 60 + seconds

    # Create a timedelta object with the appropriate sign
    utc_time_delta = datetime.timedelta()
    if sign == '+':
        utc_time_delta = datetime.timedelta(seconds=total_seconds)
    else:
        utc_time_delta = -datetime.timedelta(seconds=total_seconds)

    sim_start_time_utc = (sim_start_time + utc_time_delta).replace(tzinfo=datetime.timezone.utc)
    sim_end_time_utc = (sim_end_time + utc_time_delta).replace(tzinfo=datetime.timezone.utc)

    # Get irradiance from Solcast Data
    api_keys = None
    # Update API key usage data
    with open(SOLCAST_KEYS_METADATA_PATH, 'r') as file:
        api_keys = json.load(file)
        if (api_keys == None):
            raise RuntimeError(f"No data in {SOLCAST_KEYS_METADATA_PATH}")
        elif (API_KEY in api_keys):
            print(f"Using API Key {API_KEY} with {api_keys[API_KEY]} uses left")
        else:
            raise RuntimeError(f"Unrecognized API key {API_KEY}")
    
    with open(SOLCAST_KEYS_METADATA_PATH, 'w') as file:
        api_keys[API_KEY] -= 1
        json_object = json.dumps(api_keys)
        file.write(json_object)
        
    # Get DNI, DHI
    solcast_data = historic_call_by_site(lat, lon, sim_start_time_utc.isoformat(), sim_end_time_utc.isoformat())
    # solcast_data = None
    # with open("./solcast_data.json", 'r') as file:
    #     solcast_data = json.load(file)['data']
    # with open("Data.json", 'w') as file:
    #     json.dump(solcast_data, file, indent=4)

    timestep = (sim_end_time - sim_start_time) / num_timesteps

    curr_time_utc = sim_start_time_utc
    curr_time = sim_start_time
    output_csv = []
    
    # Get irradiance, azimuth, elevation for each timestamp
    index_cache = 0
    for _ in range(0, num_timesteps):
        az = get_azimuth(lat, lon, curr_time_utc)  

        # During morning and evening times, assume the array is directory normal to the sun
        el = None
        if (compare_clock_time(RACE_START_TIME, curr_time) or compare_clock_time(curr_time, RACE_END_TIME)):
            el = 90.0
        else:
            el = get_altitude(lat, lon, curr_time_utc)

        # Get irradiance
        data_point, index_cache = get_irradiance(solcast_data, curr_time_utc, index_cache)
        output_csv.append([az, el, data_point['dni'], curr_time.strftime('%Y-%m-%d %H:%M:%S')])

        curr_time_utc += timestep
        curr_time += timestep
    
    # Write data to csv
    with open(output_csv_name, 'w', newline='') as csv_file:
        # Create a CSV writer object
        csv_writer = csv.writer(csv_file)

        # Write the data to the CSV file
        csv_writer.writerows(output_csv)

if __name__ == "__main__":
    parser = ArgumentParser(description='Create solar irradiance simulation csv',
                            epilog='Example: dayAzElIrr.py --lat 40.791267 --lon -111.734322 --start_time "2024-07-10 08:00:00" --end_time "2024-07-10 20:00:00" --utc_adjustment +06:00:00 --num_timesteps 180')
    parser.add_argument('--lat', type=float, required=True, help='Latitude of the static location in degrees')
    parser.add_argument('--lon', type=float, required=True, help='Longitude of the static location in degrees')
    parser.add_argument('--start_time', type=str, required=True, help='Start time of the simulation in YYYY-MM-DD HH:MM:SS format in 24 hour local time')
    parser.add_argument('--end_time', type=str, required=True, help='End time of the simulation in YYYY-MM-DD HH:MM:SS format in 24 hour local time')
    parser.add_argument('--utc_adjustment', type=str, required=True, help='Adjustment in hours and minutes from local time to UTC in (+/-)HH:MM:SS format')
    parser.add_argument('--num_timesteps', type=int, required=True, help='Number of timestamps to generate data for')
    parser.add_argument('--out_csv', type=str, default='sun_position.csv', help='Name of output CSV file [default=sun_position.csv]')
    args = parser.parse_args()

    generate_irr_csv(args.lat, args.lon, args.start_time, args.end_time, args.utc_adjustment, args.num_timesteps, args.out_csv)
