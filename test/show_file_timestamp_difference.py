# Calculate and display the time difference in seconds between consecutive files in a specified directory 
# (or the current working directory if no directory is specified). 
#
#   * Command-Line Argument Handling: It checks if a directory path is provided as a command-line argument. 
#     If not, it defaults to the current working directory.
#   * File Listing: It generates a list of all files in the specified directory, sorted by their names.
#   * Time Difference Calculation: For each pair of consecutive files, it calculates the difference in their last modification times in seconds.
#   * Output: It prints the calculated time difference for each pair of files, along with their names.
#
#
#  @author: BSIT, Andrea Novati - N3 S.r.l. 

import os
import datetime
import sys
from glob import glob

# Check if a command-line argument is provided for the directory path
if len(sys.argv) > 1:
    directory_path = sys.argv[1]  # Use the provided argument as the directory path
else:
    directory_path = os.getcwd()  # Use the current working directory

# Get all files in the directory, sorted by name
files = sorted(glob(os.path.join(directory_path, "*")))

# Iterate over the files, calculating the time difference between each file and the next
for i in range(len(files) - 1):
    # Get the modification time of the current file and the next file
    current_time = os.path.getmtime(files[i])
    next_time = os.path.getmtime(files[i + 1])

    # Convert times to datetime objects
    current_time_dt = datetime.datetime.fromtimestamp(current_time)
    next_time_dt = datetime.datetime.fromtimestamp(next_time)

    # Calculate the difference in seconds
    diff_in_seconds = (next_time_dt - current_time_dt).total_seconds()

    # Print the difference
    print(f"Difference between {os.path.basename(files[i])} and {os.path.basename(files[i + 1])} is {diff_in_seconds} seconds.")
