import argparse
import os
import random
from typing import Any

import numpy as np
import matplotlib.pyplot as plt
from numpy import ndarray

from PIL import Image
from numpy.fft import fft2, ifft2, fftshift, ifftshift
# import cupy as cp
# from cupy.fft import fft2, ifft2, fftshift
import sys

mm = 1
um = 1e-3
nm = 1e-6

wl_red = 632.8 * nm
wl_green = 532 * nm
wl_blue = 441.563 * nm


# Import CGH
def import_cgh(image_path: str, grayscale=False, phase_only=False, rgb_only=False) -> list[ndarray[Any]]:
    if image_path.endswith(".csv"):  # CSV Complex format
        data_im = np.recfromtxt(f"./{image_path}", delimiter=",", names=None)
        print(data_im.dtype)
        complex_data = data_im
    elif image_path.endswith(".mat"):  # MATLAB Complex format
        from scipy.io import loadmat
        data_im = loadmat(f'./{image_path}')
        complex_data = data_im['data']
    elif image_path.endswith(".bin"):  # Raw binary Complex format
        data_im = np.fromfile(f"./{image_path}", dtype=complex)
        if data_im.shape[0] == 1080 * 1920 * 2 * 2:
            complex_data = np.reshape(data_im, (1080 * 2, 1920 * 2))
        elif data_im.shape[0] == 1080 * 1920:
            complex_data = np.reshape(data_im, (1080, 1920))
        elif data_im.shape[0] == 400 * 600:
            complex_data = np.reshape(data_im, (400, 600))
        else:
            print(f"Unknown image size for {image_path}: {data_im.shape[0]}")
            exit(1)
        print(f"Image size: {complex_data.shape[0]}x{complex_data.shape[1]}")
        if phase_only:
            complex_data = np.exp(1j * np.angle(complex_data))
    elif image_path.endswith(".png"):  # PNG Phase format (grayscale and rgb[a])
        image_data = Image.open(f'./{image_path}')
        data_im = np.array(image_data)
        if data_im.ndim == 1 or grayscale:  # Grayscale
            if data_im.ndim != 1:
                data_im = data_im[:, :, 0]
                print("Image is not grayscale, but grayscale flag is set. Converting to grayscale.")
            else:
                print("Image is grayscale, creating a grayscale reconstruction.")

            data_norm = (data_im - 127.5) / 127.5
            complex_data = np.exp(1j * np.pi * data_norm)
            return [complex_data]
        elif data_im.shape[2] == 4:  # RGBA
            data_norm = (data_im[:, :, 0] - 127.5) / 127.5
            complex_data_r = np.exp(1j * np.pi * data_norm)
            data_norm = (data_im[:, :, 1] - 127.5) / 127.5
            complex_data_g = np.exp(1j * np.pi * data_norm)
            data_norm = (data_im[:, :, 2] - 127.5) / 127.5
            complex_data_b = np.exp(1j * np.pi * data_norm)
            if rgb_only:
                print("Image is RGBA, but rgb_only flag is set. Converting to RGB.")
                return [complex_data_r, complex_data_g, complex_data_b]
            else:
                print("Image is RGBA, creating a color and luminance reconstruction.")
                data_norm = (data_im[:, :, 3] - 127.5) / 127.5
                complex_data_a = np.exp(1j * np.pi * data_norm)
                return [complex_data_r, complex_data_g, complex_data_b, complex_data_a]
        elif data_im.shape[2] == 3:  # RGB
            print("Unsupported operation. Use RGBA format instead.")
            exit(1)
            # print("Image is RGB, creating a color reconstruction.")
            # data_norm = (data_im[:, :, 0] - 127.5) / 127.5
            # complex_data_r = np.exp(1j * np.pi * data_norm)
            # data_norm = (data_im[:, :, 1] - 127.5) / 127.5
            # complex_data_g = np.exp(1j * np.pi * data_norm)
            # data_norm = (data_im[:, :, 2] - 127.5) / 127.5
            # complex_data_b = np.exp(1j * np.pi * data_norm)
            # return [complex_data_r, complex_data_g, complex_data_b]
    else:
        print("Unknown image format. Supported formats are: PNG, CSV, MAT, and BIN.")
        exit(1)

    return [complex_data]


Image.MAX_IMAGE_PIXELS = None


# Propagation kernel
def propagate_angular(data: ndarray[complex], slm_z: float, wavelength: float, virtual_slm_factor=1):
    print(f"params: slm_z={slm_z}, wavelength={wavelength}, virtual_slm_factor={virtual_slm_factor}")
    # Have margins for the fft
    # data = cp.array(data, dtype=complex, blocking=True)
    nx = 2048 * 2 * 2
    ny = 2048 * 2 * 2

    # Physical slm size
    pixel_size = 8 * um / virtual_slm_factor
    lx = nx * pixel_size
    ly = ny * pixel_size

    fx = (wavelength / lx) * (np.arange(nx) - nx // 2)
    fy = (wavelength / ly) * (np.arange(ny) - ny // 2)

    fxx, fyy = np.meshgrid(fx, fy)

    mod_fxfy = fxx * fxx + fyy * fyy
    k = 2 * np.pi / wavelength
    steps = 1
    a = fft2(data, (nx, ny))
    for i in range(steps):
        kernel = np.exp(1j * ((k * slm_z / steps) * np.sqrt(1 - mod_fxfy)))
        kernel = fftshift(kernel)
        a = ifft2(a * kernel)
        if i == steps - 1: break
        a = fft2(a + np.exp(1j * random.random() * 2 * np.pi))

    propagated = a[:1080 * virtual_slm_factor, :1920 * virtual_slm_factor]

    return propagated


def propagate_fresnel(data: ndarray[complex], slm_z: float, wavelength: float, virtual_slm_factor):
    # Fresnel
    nx = 2048 * 2
    ny = 2048 * 2

    # Physical slm size
    pixel_size = 8 * um

    fx = np.fft.fftfreq(nx, d=8 * um / virtual_slm_factor)
    fy = np.fft.fftfreq(ny, d=8 * um / virtual_slm_factor)
    fxx, fyy = np.meshgrid(fx, fy)

    k = 2 * np.pi / wavelength
    kernel = np.exp(1j * k * slm_z) * np.exp(-1j * np.pi * wavelength * slm_z * (fxx ** 2 + fyy ** 2))

    data_im = fft2(data, (nx, ny))
    fresnel_image = ifft2(data_im * kernel)
    return fresnel_image[:1080 * virtual_slm_factor, :1920 * virtual_slm_factor]


def normalize(data: ndarray):
    return data / np.max(np.abs(data))


def propagate(data: ndarray[complex], slm_z: float, wavelength: float, virtual_slm_factor=1):
    out = propagate_angular(data, slm_z, wavelength, virtual_slm_factor)

    return out


def plot_image(image: ndarray):
    plt.axis('off')
    plt.axes([0.0, 0.0, 1.0, 1.0])
    plt.axis('off')
    plt.imshow(image)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Propagate a CGH")
    parser.add_argument("CGH", type=str, help="Path to the CGH file in PNG, CSV, MAT, or BIN format")
    parser.add_argument("-g", "--grayscale", action="store_true", help="Grayscale image")
    parser.add_argument("-p", "--phase_only", action="store_true", help="Phase only image")
    parser.add_argument("-c", "--count", type=int, default=1, help="Number of images to propagate")
    parser.add_argument("-rgb", "--rgb", action="store_true", help="Only RGB image")
    parser.add_argument("-z", "--z", type=float, default=291, help="Z distance to propagate")

    if len(sys.argv) == 1:
        parser.print_help()
        exit(0)

    args = parser.parse_args()

    image_path = args.CGH
    grayscale = args.grayscale
    phase_only = args.phase_only
    count = args.count
    rgb_only = args.rgb
    z = args.z

    # For multiple images
    if count > 1:
        # if not os.path.exists(f"{image_path}/average"): os.mkdir(f"{image_path}/average")
        # if not os.path.exists(f"{image_path}/median"): os.mkdir(f"{image_path}/median")
        if not os.path.exists(f"{image_path}/out"): os.mkdir(f"{image_path}/out")

        import concurrent.futures

        def process_image(i):

            print(f"Image {i}:")
            if grayscale:
                complex_data = import_cgh(f"{image_path}/{i}.png", grayscale=True, phase_only=phase_only)
                r = (np.abs(propagate(complex_data[0], -z * mm, wl_red)))
                plt.imsave(f"{image_path}/out/{i}.png", r, cmap='gray')
            else:
                complex_data = import_cgh(f"{image_path}/{i}.png", grayscale=False, phase_only=True, rgb_only=False)
                r = (np.abs(propagate(complex_data[0], -z * mm, wl_red)) / 7).clip(0, 1)
                g = (np.abs(propagate(complex_data[1], -z * mm, wl_green)) / 7).clip(0, 1)
                b = (np.abs(propagate(complex_data[2], -z * mm, wl_blue)) / 7).clip(0, 1)
                a = (np.abs(propagate(complex_data[3], -z * mm, wl_red)) / 7).clip(0, 1)
                plt.imsave(f"{image_path}/out/{i}.png", np.dstack((r, g, b)))
                plt.imsave(f"{image_path}/out/{i}g.png", a, cmap='gray')

        # Parallelize the loop
        with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() - 2) as executor:
            executor.map(process_image, range(0, count))

        return

    # Import CGH
    complex_data = import_cgh(image_path, grayscale, phase_only)
    if rgb_only: print("TODO: rgb_only flag in output is not implemented.")

    def imsave_grayscale(z):
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_{z}.png',
                   np.abs(propagate(complex_data[0], -z * mm, wl_red)),
                   cmap='gray')

    propagate_range = False
    if propagate_range:
        for z in range(2900, 3025, 5):
            # for z in range(3005, 3105, 5):
            z /= 10
            r = (np.abs(propagate(complex_data[0], -z * mm, wl_red)))
            g = (np.abs(propagate(complex_data[1], -z * mm, wl_green)))
            b = (np.abs(propagate(complex_data[2], -z * mm, wl_blue)))

            r /= 2
            g /= 2
            b /= 2

            rgb = np.dstack((r, g, b))
            plt.imsave(f'output/propagation/color/range/{z * 10}.png', rgb.clip(0, 1))
        return

    # Grayscale
    if len(complex_data) == 1:
        # plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_{z}.png', np.abs(propagate(complex_data[0], -z * mm, wl_red)), cmap='gray')
        plt.figure(figsize=(16, 9))
        plt.margins(0)
        plt.tight_layout(pad=0)
        plt.axis('off')
        a = np.abs(propagate(complex_data[0], -z * mm, wl_red))
        plt.imsave("out.png", a, cmap='gray')
        plt.imshow(a, cmap='gray')
        plt.show()
        # imsave_grayscale(292)
        # imsave_grayscale(300)
        # imsave_grayscale(306)

    # Color
    else:
        r = (np.abs(propagate(complex_data[0], -z * mm, wl_red)))
        g = (np.abs(propagate(complex_data[1], -z * mm, wl_green)))
        b = (np.abs(propagate(complex_data[2], -z * mm, wl_blue)))
        a = (np.abs(propagate(complex_data[3], -z * mm, wl_red)))

        r /= 2
        g /= 2
        b /= 2
        a /= 2

        rgb = np.dstack((r, g, b))
        zero = np.zeros_like(r)
        fig, axes = plt.subplots(2, 2, figsize=(16, 9))
        data = [rgb, np.dstack((r, zero, zero)), np.dstack((zero, g, zero)), np.dstack((zero, zero, b))]
        for i, ax in enumerate(axes.flat):
            ax.axis('off')
            ax.imshow(data[i])

        plt.margins(0)
        plt.tight_layout(pad=0)
        plt.savefig("output/propagation/color/figure.png")
        # plt.figure("Luminance")
        # plt.imshow(np.dstack((a, a, a)).clip(0, 1), cmap='gray')
        #
        plt.show()
        plt.figure(figsize=(16, 9))
        plt.axis("off")
        plt.margins(0)
        plt.tight_layout(pad=0)
        plt.imshow(rgb.clip(0, 1))
        plt.show()
        plt.imsave('output/propagation/color/rgb.png', rgb.clip(0, 1))
        plt.imsave('output/propagation/color/r.png', np.dstack((r, zero, zero)).clip(0, 1))
        plt.imsave('output/propagation/color/g.png', np.dstack((zero, g, zero)).clip(0, 1))
        plt.imsave('output/propagation/color/b.png', np.dstack((zero, zero, b)).clip(0, 1))
        plt.imsave('output/propagation/color/a.png', np.dstack((a, a, a)).clip(0, 1))

        exit()
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_300.png', np.dstack((r, g, b)))
        print(f"Image saved as 'output/propagation/{image_path.split('/')[-1]}_300.png'")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
