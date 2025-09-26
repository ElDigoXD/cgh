import os
from pprint import pprint

import numpy as np
import PIL.Image
import argparse
import sys

parser = argparse.ArgumentParser(description="Propagate, average, calibrate and calculate CC")
parser.add_argument("Path", type=str, help="Path to the folder")

args = parser.parse_args()

files = [x for x in os.listdir(args.Path + "/src/") if os.path.isfile(args.Path + "/src/" + x)]
files.sort()
n = files.__len__()
print("Number of images:", n)

# Propagate
if not os.path.exists(args.Path + "/src/out/"):
    pass

# Average
if not os.path.exists(args.Path + "/average_1.png"):
    print("Averaging")
    import subprocess
    [subprocess.call(["bash", "-c", f"convert {args.Path}/src/out/{{0..{i}}}.png  -evaluate-sequence mean {args.Path}/average_{i+1}.png"]) for i in range(n)]
    [(subprocess.call(["bash", "-c", f"convert {args.Path}/src/out/{{0..{i}}}g.png  -evaluate-sequence mean {args.Path}/g_average_{i+1}.png"]) , print(i, " ", end="")) for i in range(n)]
    print("\nDone grayscale")
# Calibrate
if not os.path.exists(args.Path + "/src/calibrado/"):
    pass

# Calculate CC
if not os.path.exists(args.Path + "/cc.txt"):
    pass