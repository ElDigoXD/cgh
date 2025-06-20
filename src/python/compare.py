import cv2
import numpy as np
import os
import re
from skimage.metrics import structural_similarity as ssim
from skimage.measure import shannon_entropy
from skimage.util import img_as_float
from tabulate import tabulate
from pypiqe import piqe

def calculate_snr(original, noisy):
    original = original.astype(np.float64)
    noisy = noisy.astype(np.float64)
    signal_power = np.mean(original ** 2)
    noise_power = np.mean((original - noisy) ** 2)
    if noise_power == 0:
        return float('inf')
    return 10 * np.log10(signal_power / noise_power)

def calculate_psnr(original, noisy):
    mse = np.mean((original.astype(np.float64) - noisy.astype(np.float64)) ** 2)
    if mse == 0:
        return float('inf')
    max_pixel = 255.0
    return 10 * np.log10((max_pixel ** 2) / mse)

def calculate_mse(original, noisy):
    return np.mean((original.astype(np.float64) - noisy.astype(np.float64)) ** 2)

def calculate_ssim(original, noisy):
    return ssim(original, noisy, data_range=255)

def calculate_std(image):
    return np.std(image)

def calculate_entropy(image):
    return shannon_entropy(image)

# Paths
original_dir = "output/color/average"
noisy_dir = "output/color/average"

# List of filenames (common to both dirs)
image_files = sorted(os.listdir(original_dir), key= lambda x: int(re.search(r'average_(\d+)\.png', x).group(1)))

# Prepare results
results = []

for filename in image_files:
    original_path = "/home/dsanz/Descargas/image (1).png"
    original_path = "output/color/average/average_128.png"
    noisy_path = os.path.join(noisy_dir, filename)

    if not os.path.exists(noisy_path):
        continue  # Skip if noisy pair doesn't exist

    original = cv2.imread(original_path, cv2.IMREAD_GRAYSCALE)
    noisy = cv2.imread(noisy_path, cv2.IMREAD_GRAYSCALE)

    snr_val = calculate_snr(original, noisy)
    psnr_val = calculate_psnr(original, noisy)
    mse_val = calculate_mse(original, noisy)
    ssim_val = calculate_ssim(original, noisy)
    std_val = calculate_std(noisy)
    entropy_val = calculate_entropy(noisy)

    # PIQE
    piqe_score, _, _, _ = piqe(noisy)

    results.append([
        filename,
        f"{snr_val:.2f}",
        f"{psnr_val:.2f}",
        f"{mse_val:.2f}",
        f"{ssim_val:.4f}",
        f"{std_val:.4f}",
        f"{entropy_val:.4f}",
        f"{piqe_score:.4f}"
    ])

# Print table
headers = ["Filename", "SNR (dB)", "PSNR (dB)", "MSE", "SSIM", "Std Dev", "Entropy", "PIQE"]
print(tabulate(results, headers=headers, tablefmt="rst"))