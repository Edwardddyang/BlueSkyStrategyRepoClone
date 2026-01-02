
import pandas as pd
from datetime import datetime, timedelta

def process_csv(input_file, output_file):
    # Read the CSV file
    df = pd.read_csv(input_file)

    # Assume the CSV has at least three columns, no header rows, and requires adding two more columns.
    # Generate a datetime sequence starting from the current time
    start_time = datetime.now()

    # Increment by 5 seconds for each row
    df['Datetime'] = [start_time + timedelta(seconds=5 * i) for i in range(len(df))]

    # Format the datetime to match 'YYYY-MM-DD HH:MM:SS'
    df['Datetime'] = df['Datetime'].dt.strftime('%Y-%m-%d %H:%M:%S')

    # Add a fifth column with the constant number 12
    df['Constant'] = 12

    # Write the modified DataFrame back to a new CSV file
    df.to_csv(output_file, index=False, header=False)

# Usage example
input_file = 'telem.csv'
output_file = 'telem2.csv'
process_csv(input_file, output_file)