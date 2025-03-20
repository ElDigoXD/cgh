from typing import Any

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from numpy import ndarray

matplotlib.use('TkAgg')
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
def import_cgh(image_path: str, grayscale=False, phase_only=False) -> list[ndarray[Any]]:
    if image_path.endswith(".csv"):  # CSV Complex format
        data_im = np.recfromtxt(f"./{image_path}", delimiter=",", names=None)
        print(data_im.dtype)
        complex_data = data_im
    elif image_path.endswith(".mat"):  # MATLAB Complex format
        from scipy.io import loadmat

        data_im = loadmat(f'./{image_path}')
        complex_data = data_im['data']
        print(complex_data.dtype)
    elif image_path.endswith(".bin"):  # Raw binary Complex format
        data_im = np.fromfile(f"./{image_path}", dtype=complex)
        if data_im.shape[0] == 1080 * 1920:
            complex_data = np.reshape(data_im, (1080, 1920))
        elif data_im.shape[0] == 400 * 600:
            complex_data = np.reshape(data_im, (400, 600))
        else:
            print(f"Unknown image size for {image_path}: {data_im.shape[0]}")
            exit(1)
        if phase_only:
            complex_data = np.exp(1j * np.angle(complex_data))
    elif image_path.endswith(".png"):  # PNG Phase format (grayscale and rgb)
        image_data = Image.open(f'./{image_path}')
        data_im = np.array(image_data)

        if data_im.ndim == 1 or grayscale:  # Grayscale
            if data_im.ndim == 3:
                data_im = data_im[:, :, 0]

            print("Image is grayscale, creating a grayscale reconstruction.")
            data_norm = (data_im - 127.5) / 127.5
            complex_data = np.exp(1j * np.pi * data_norm)
            return [complex_data]
        else:  # RGB
            print("Image is RGB, creating a color reconstruction.")
            data_norm = (data_im[:, :, 0] - 127.5) / 127.5
            complex_data_r = np.exp(1j * np.pi * data_norm)
            data_norm = (data_im[:, :, 1] - 127.5) / 127.5
            complex_data_g = np.exp(1j * np.pi * data_norm)
            data_norm = (data_im[:, :, 2] - 127.5) / 127.5
            complex_data_b = np.exp(1j * np.pi * data_norm)
            return [complex_data_r, complex_data_g, complex_data_b]
    else:
        print("Unknown image format.")
        exit(1)

    return [complex_data]


# Propagation kernel


def propagate(data: ndarray[complex], slm_z: float, wavelength: float):
    # Have margins for the fft
    nx = 2048*2
    ny = 2048*2

    # Physical slm size
    pixel_size = 8 * um
    lx = nx * pixel_size
    ly = ny * pixel_size

    fx = (wavelength / lx) * (np.arange(nx) - nx // 2)
    fy = (wavelength / ly) * (np.arange(ny) - ny // 2)

    fxx, fyy = np.meshgrid(fx, fy)

    mod_fxfy = fxx * fxx + fyy * fyy
    k = 2 * np.pi / wavelength
    kernel = np.exp(1j * ((k * slm_z) * np.sqrt(1 - mod_fxfy)))
    kernel = fftshift(kernel)

    propagated = ifft2((fft2(data, (nx, ny)) * kernel))[:1080, :1920]

    return propagated


"""
def plot():
    # Show an image with pyplot
    zs = range(600, 1001, 50)
    zs = range(760, 850, 10)
    zs = range(200, 218, 2)
    zs = [299, 300, 301]
    zs = range(50, 600, 50)
    zs = range(290, 310, 2)

    max_columns = 3
    num_rows = int(np.ceil(len(zs) / max_columns))
    num_cols = min(len(zs), max_columns)

    print(f"Plotting {len(zs)} images in a grid of {num_rows} rows and {num_cols} columns.")

    fig, axes = plt.subplots(
        num_rows, num_cols, figsize=(16 * num_cols // 2, 9 * num_rows // 2))
    if num_rows == 1:
        axes = np.reshape(axes, (1, -1))
    elif num_cols == 1:
        axes = np.reshape(axes, (-1, 1))

    for i, ax in enumerate(axes.flat):
        if i < len(zs):
            ax.imshow(np.abs(propagate(-zs[i] * mm)), cmap='gray')
            ax.axis('off')
            ax.set_title('z = {} mm'.format(zs[i]))
            width = data_im.shape[1]
            height = data_im.shape[0]
            zoom_factor = 1
            x_center, y_center = width // 2, height // 2  # Center of the image
            zoom_width, zoom_height = width // zoom_factor, height // zoom_factor
            ax.set_xlim(x_center - zoom_width // 2, x_center + zoom_width // 2)
            ax.set_ylim(y_center + zoom_height // 2, y_center - zoom_height // 2)  # Inverted y-axis for images
        else:
            ax.axis('off')

    plt.tight_layout()
    plt.savefig("output/propagation/figure.png")
    plt.show()
"""

def main():
    if len(sys.argv) == 1:
        print(f"Usage: python {sys.argv[0]} <image.[png|csv|mat|bin]>")
        exit(1)
    image_path = sys.argv[1]

    # Import CGH
    complex_data = import_cgh(image_path, grayscale=True)

    def imsave_grayscale(z):
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_{z}.png',
                   np.abs(propagate(complex_data[0], -z * mm, wl_red)),
                   cmap='gray')

    if len(complex_data) == 1:  # Grayscale
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_290.png', np.abs(propagate(complex_data[0], -290 * mm, wl_red)), cmap='gray')
        plt.imshow(np.abs(propagate(complex_data[0], -290 * mm, wl_red)), cmap='gray')
        plt.show()
        #imsave_grayscale(292)
        #imsave_grayscale(300)
        #imsave_grayscale(306)
    else:
        r = (np.abs(propagate(complex_data[0], -290 * mm, wl_red)))
        g = (np.abs(propagate(complex_data[1], -290 * mm, wl_green)))
        b = (np.abs(propagate(complex_data[2], -290 * mm, wl_blue)))
        # Normalize to 0-1 with the maximum value of the three channels
        max_val = np.max([np.max(r), np.max(g), np.max(b)])
        min_val = np.min([np.min(r), np.min(g), np.min(b)])
        r = (r - min_val) / (max_val - min_val)
        g = (g - min_val) / (max_val - min_val)
        b = (b - min_val) / (max_val - min_val)

        r = r
        g = g
        b = b


        rgb = np.dstack((r, g, b))
        # todo: luminance in A
        zero = np.zeros_like(r)
        fig, axes = plt.subplots(2, 2, figsize=(16, 9))
        data = [rgb, np.dstack((r, zero, zero)), np.dstack((zero, g, zero)), np.dstack((zero, zero, b))]
        for i, ax in enumerate(axes.flat):
            ax.axis('off')
            ax.imshow(data[i])

        plt.margins(0)
        plt.tight_layout(pad=0)
        plt.savefig("output/propagation/color/figure.png")
        plt.show()
        plt.imsave('output/propagation/color/rgb.png', rgb.clip(0, 1))
        plt.imsave('output/propagation/color/r.png', np.dstack((r, zero, zero)).clip(0, 1))
        plt.imsave('output/propagation/color/g.png', np.dstack((zero, g, zero)).clip(0, 1))
        plt.imsave('output/propagation/color/b.png', np.dstack((zero, zero, b)).clip(0, 1))
        plt.imsave("output/propagation/color/luminance.png", (np.abs(propagate(import_cgh(image_path.replace(".png", ".bin"), phase_only=True)[0], -290 * mm, wl_red))), cmap='gray')

        exit()
        plt.imsave(f'output/propagation/{image_path.split("/")[-1]}_300.png', np.dstack((r, g, b)))
        print(f"Image saved as 'output/propagation/{image_path.split('/')[-1]}_300.png'")


if __name__ == "__main__":
    main()
