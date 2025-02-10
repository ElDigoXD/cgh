import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')
from PIL import Image
from numpy.fft.helper import fftshift
from numpy.fft import fft2, ifft2
import sys

image = "ph_1gpu_1638s.png"
if len(sys.argv) > 1:
    image = sys.argv[1]

mm = 1
um = 1e-3
nm = 1e-6

pixelsize = 8 * um
wl = 632.8 * nm
k = 2 * np.pi / wl

# z_srt = input('Propagation distance, z (mm): ')

# lx = 15.36 * mm
# ly = 8.64 * mm

dx = pixelsize
dy = pixelsize

# Import CGH
if image.endswith(".csv"):
    data_im = np.recfromtxt(f"./{image}", delimiter=",", names=None)
    print(data_im.dtype)
    complex_data = data_im
elif image.endswith(".mat"):
    from scipy.io import loadmat
    data_im = loadmat(f'./{image}')
    complex_data = data_im['data']
    print(complex_data.dtype)
elif image.endswith(".bin"):
    data_im = np.fromfile(f"./{image}", dtype=complex)
    if data_im.shape[0] == 1080 * 1920:
        complex_data = np.reshape(data_im, (1080, 1920))
    elif data_im.shape[0] == 400 * 600:
        complex_data = np.reshape(data_im, (400, 600))
    else:
        print("Unknown image size.")
        exit(1)
else:
    image_data = Image.open(f'./{image}')
    data_im = np.array(image_data)

    if data_im.ndim == 3:
        data_im = data_im[:, :, 0]
    print("Imported image: ", data_im.shape, " pixels.")

    # [0, 255] --> [-1, 1]
    data_norm = (data_im - 127.5) / 127.5

    # [-1, 1] --> [-pi, pi]
    complex_data = np.exp(1j * np.pi * data_norm)
    print(np.max(complex_data))
    print(np.min(complex_data))

nx = int(complex_data.shape[1])
ny = int(complex_data.shape[0])

nx = 2048
ny = 2048

lx = nx * dx
ly = ny * dy


# Propagation kernel


def propagation_kernel(slm_z):
    fx = (wl / lx) * (np.arange(nx) - nx // 2)
    fy = (wl / ly) * (np.arange(ny) - ny // 2)

    fxx, fyy = np.meshgrid(fx, fy)

    mod_fxfy = fxx * fxx + fyy * fyy
    kernel = np.exp(1j * ((k * slm_z) * np.sqrt(1 - mod_fxfy)))
    kernel = fftshift(kernel)

    propagated = ifft2((fft2(complex_data, (2048, 2048)) * kernel))[:1080, :1920]

    return propagated

def imsave(z):
    plt.imsave(f'output/propagation/{image.split("/")[-1]}_{z}.png', np.sqrt(np.abs(propagation_kernel(-z * mm))), cmap='gray')

imsave(294)
imsave(300)
imsave(306)
exit(0)
# Show an image with pyplot

zs = range(600, 1001, 50)
zs = range(760, 850, 10)
zs = range(200, 218, 2)
zs = [299, 300, 301]
zs = range(50, 600, 50)
zs = range(290, 310 , 2)

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
        ax.imshow(np.abs(propagation_kernel(-zs[i] * mm)), cmap='gray')
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
