#!/usr/bin/env python3
""" Removes uneccessary dependancies from release branches """
import shutil
import os

files = {
    "CMakeLists.txt": [16, 28],
    "package.xml": [26]
}

for filename in files.keys():
    # Make copy of file
    shutil.move(filename, filename + "~")

    destination = open(filename, "w")
    source = open(filename + "~", "r")

    try:
        for i, line in enumerate(source):
            if i in files[filename]:
                continue
            destination.write(line)
        source.close()
        destination.close()
        os.remove(filename + "~")
    except:
        source.close()
        destination.close()
        shutil.move(filename + "~", filename)
