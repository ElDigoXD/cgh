import os
import sys

import PIL.Image
import argparse
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description="Calibrate CGHs")
parser.add_argument("CGH", type=str, help="Path to the CGH directory")
parser.add_argument("-c", "--count", type=int, default=1, help="Number of images to propagate")

args = parser.parse_args()
if len(sys.argv) == 1:
    parser.print_help()
    exit(1)

image_path = args.CGH
count = args.count

if not os.path.exists(f"{image_path}/calibrado"): os.mkdir(f"{image_path}/calibrado")

for i in range(count):
    # Carga la imagen
    imagen_raw = PIL.Image.open(f"{image_path}/{i}.png")
    a = np.array(imagen_raw)
    rango = 255
    
    # Calculo de nueva fase, calibrada para el SLM
    a = a[:, :, 3]
    phase = a * 2.0 * np.pi / rango + np.pi / 2.0

    # Coeficientes obtenidos a partir del proceso de calibracion
    p1 = -0.0058
    p2 = 0.4810
    p3 = -4.6128
    p4 = 29.2003
    p5 = -32.1460

    # Nuevos niveles de grises para la imagen (en double)
    NG = p1 * (phase**4) + p2 * (phase**3) + p3 * (phase**2) + p4 * phase + p5

    # Conversion de double a enteros en formato 8 bits
    NG2 = np.int8(NG)

    # Almacenamiento del holograma calibrado
    plt.imsave(f"{image_path}/calibrado/{i}.png", NG2, cmap="gray",  vmin=0, vmax=255)