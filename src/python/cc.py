import os
from pprint import pprint

import numpy as np
import PIL.Image
import argparse
import sys

from tabulate import tabulate


def get_cc(a, b):
    return np.corrcoef(a.flatten(), b.flatten())[0, 1]
def get_rmse(a, b):
    return np.sqrt(np.mean((a - b) ** 2))

def get_snr(a, b):
    original = a.astype(np.float64)
    noisy = b.astype(np.float64)
    signal_power = np.mean(original ** 2)
    noise_power = np.mean((original - noisy) ** 2)
    if noise_power == 0:
        return float('inf')
    return 10 * np.log10(signal_power / noise_power)

def get_psnr(a, b):
    mse = np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)
    if mse == 0:
        return float('inf')
    max_pixel = 255.0
    return 10 * np.log10((max_pixel ** 2) / mse)

parser = argparse.ArgumentParser(description="Calculate CC of two images")
parser.add_argument("Reference image", type=str, help="Path to the reference image")
parser.add_argument("Other images", type=str, help="Path to the image's path")

args = parser.parse_args()
if len(sys.argv) == 1:
    parser.print_help()
    exit(1)

reference_image_path = args.__dict__["Reference image"]
image_path = args.__dict__["Other images"]

imagen_raw = PIL.Image.open(reference_image_path).convert('L')
reference = np.array(imagen_raw)
results = []
for path in os.listdir(image_path):
    if not path.endswith(".png") or not path.endswith("crop.png") or path.endswith("dragon.png") or path.startswith("g"):
        continue
    # Carga la imagen
    imagen_raw = PIL.Image.open(image_path+path).convert('L')
    other = np.array(imagen_raw)
    cc = get_cc(reference, other)
    rmse = get_rmse(reference, other)
    snr = get_snr(reference, other)
    #print(path, cc)
    n = path.replace("average_", "").replace(".png", "").replace("g_", "").replace("_crop", "").replace("_dragon_only", "").replace("_dragon", "")
    results.append([path, int(n), cc, rmse, snr])

headers = ["Filename", "N", "CC", "RMSE", "SNR (dB)"]
results = sorted(results, key=lambda x: x[1])
print(tabulate(results, headers=headers, tablefmt="rst"))

[print(x[2]) for x in results]


