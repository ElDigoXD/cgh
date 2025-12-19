import argparse
import subprocess

parser = argparse.ArgumentParser(description="Average")
parser.add_argument("Path", type=str, help="Path to the root folder")

args = parser.parse_args()


for i in range(2900, 3030, 2):
    name = f"{i:.1f}"
    print(name)
    subprocess.call(["bash", "-c", f"convert {args.Path}/{{0..9}}_2/{name}.png  -evaluate-sequence mean {args.Path}/{name}.png"])
