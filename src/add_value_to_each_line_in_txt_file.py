#!/usr/bin/env python



with open('times.txt') as times_file:
    for time in times_file:
        # Open a file with access mode 'a'
        file_object = open('times_synced.txt', 'a')
        # Append 'hello' at the end of file
        file_object.write(str(float(time) + 1317384506.40)+"\n")
        # Close the file
        file_object.close()