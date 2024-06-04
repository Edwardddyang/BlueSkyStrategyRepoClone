import csv
import pandas as pd
import argparse

def main():
    df = pd.read_csv("./luts/wsc_2023/static/GunPointTesting.csv")
    columns_to_keep = ['Y', 'X', 'ele']

    df = df[columns_to_keep]
    df['ele'] = df['ele']/1000
    df.to_csv('new.csv', index=False)

main()