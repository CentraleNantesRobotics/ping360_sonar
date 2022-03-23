#!/usr/bin/env python3
""" Updates package version using git tag"""
import shutil
import os
import sys

files = {
    "package.xml": [3]
}

for filename in files.keys():
    # Make copy of file
    shutil.move(filename, filename + "~")

    destination = open(filename, "w")
    source = open(filename + "~", "r")

    try:
        for i, line in enumerate(source):
            if i in files[filename]:
                destination.write("  <version>{}</version> \r\n".format(sys.argv[1]))
            else:
                destination.write(line)
        source.close()
        destination.close()
        os.remove(filename + "~")
    except:
        source.close()
        destination.close()
        shutil.move(filename + "~", filename)
