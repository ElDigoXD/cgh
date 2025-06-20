import argparse
import os
from typing import Any

import numpy as np
import matplotlib.pyplot as plt
from numpy import ndarray

from PIL import Image
from numpy.fft.helper import fftshift
from numpy.fft import fft2, ifft2
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


# Propagation kernel
def propagate(data: ndarray[complex], slm_z: float, wavelength: float, virtual_slm_factor=1):
    # Have margins for the fft
    nx = 2048 * 2
    ny = 2048 * 2

    # Physical slm size
    pixel_size = 8 * um / virtual_slm_factor
    lx = nx * pixel_size
    ly = ny * pixel_size

    fx = (wavelength / lx) * (np.arange(nx) - nx // 2)
    fy = (wavelength / ly) * (np.arange(ny) - ny // 2)

    fxx, fyy = np.meshgrid(fx, fy)

    mod_fxfy = fxx * fxx + fyy * fyy
    k = 2 * np.pi / wavelength
    kernel = np.exp(1j * ((k * slm_z) * np.sqrt(1 - mod_fxfy)))
    kernel = fftshift(kernel)

    propagated = ifft2((fft2(data, (nx, ny)) * kernel))[:1080 * virtual_slm_factor, :1920 * virtual_slm_factor]

    return propagated


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

    args = parser.parse_args()
    if len(sys.argv) == 1:
        parser.print_help()
        exit(1)

    image_path = args.CGH
    grayscale = args.grayscale
    phase_only = args.phase_only
    count = args.count
    rgb_only = args.rgb
    z = args.z

    # For multiple images
    if count > 1:
        if not os.path.exists(f"{image_path}/average"): os.mkdir(f"{image_path}/average")
        if not os.path.exists(f"{image_path}/median"): os.mkdir(f"{image_path}/median")

        #for z in range(2900, 3005, 5):
        if True:
            #z /= 10
            z = 291
            rgbs = []
            for i in range(count):
                print(f"Image {i}:")
                complex_data = import_cgh(f"{image_path}/{i}.png", grayscale=False, phase_only=True, rgb_only=True)
                r = (np.abs(propagate(complex_data[0], -z * mm, wl_red)) / 7).clip(0, 1)
                g = (np.abs(propagate(complex_data[1], -z * mm, wl_green)) / 7).clip(0, 1)
                b = (np.abs(propagate(complex_data[2], -z * mm, wl_blue)) / 7).clip(0, 1)

                plt.imsave(f"{image_path}/out/{i}.png", np.dstack((r, g, b)))

                #rgbs.append(np.dstack((r, g, b)))

            return
            rgbs = np.array(rgbs)

            img = np.average(rgbs, axis=0)
            plt.imsave(f"{image_path}/average_z{z*10}.png", img)
            # plot_image(img)

            img = np.median(rgbs, axis=0)
            plt.imsave(f"{image_path}/median_z{z*10}.png", img)
            # plot_image(img)

        return

    # Import CGH
    complex_data = import_cgh(image_path, grayscale, phase_only)
    if rgb_only: print("TODO: rgb_only flag in output is not implemented.")

    def imsave_grayscale(z):
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_{z}.png',
                   np.abs(propagate(complex_data[0], -z * mm, wl_red)),
                   cmap='gray')

    if len(complex_data) == 1:  # Grayscale
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_291.png',
                   np.abs(propagate(complex_data[0], -290 * mm, wl_red)), cmap='gray')
        plt.imshow(np.abs(propagate(complex_data[0], -290 * mm, wl_red)), cmap='gray')
        plt.show()
        # imsave_grayscale(292)
        # imsave_grayscale(300)
        # imsave_grayscale(306)
    else:
        r = (np.abs(propagate(complex_data[0], -290 * mm, wl_red)))
        g = (np.abs(propagate(complex_data[1], -290 * mm, wl_green)))
        b = (np.abs(propagate(complex_data[2], -290 * mm, wl_blue)))
        a = (np.abs(propagate(complex_data[3], -290 * mm, wl_red)))
        # Normalize to 0-1 with the maximum value of the three channels
        # max_val = np.max([np.max(r), np.max(g), np.max(b)])
        # min_val = np.min([np.min(r), np.min(g), np.min(b)])
        # r = (r - min_val) / (max_val - min_val)
        # g = (g - min_val) / (max_val - min_val)
        # b = (b - min_val) / (max_val - min_val)

        r /= 7
        g /= 7
        b /= 7
        a /= 7

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
