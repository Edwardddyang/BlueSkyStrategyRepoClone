import pandas as pd
import argparse

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--lat_column", type=str, default ="Y", help="Name of the latitude column")
  parser.add_argument("--lon_column", type=str, default="X", help="Name of the longitude column")
  parser.add_argument("--alt_column", type=str, default="ele", help="Name of the altitude column")
  parser.add_argument("--convert_km", action="store_true", help="Convert altitude from m to km")
  parser.add_argument("--csv", type=str, default=None, help="Path to csv")
  parser.add_argument("--output_csv", type=str, default="out.csv", help="Output csv name")

  args = parser.parse_args()
  if args.csv is None:
    raise RuntimeError("No csv provided")
  
  df = pd.read_csv(args.csv)
  columns_to_keep = [args.lat_column, args.lon_column, args.alt_column]

  df = df[columns_to_keep]

  if (args.convert_km):
    df[args.alt_column] = df[args.alt_column] / 1000

  df.to_csv(args.output_csv, index=False, header=False)
