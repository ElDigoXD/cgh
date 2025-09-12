import cv2
import numpy as np
from matplotlib import pyplot as plt

# Cargar las dos imágenes
# Deben tener el mismo tamaño, de lo contrario habrá que redimensionarlas
img1 = cv2.imread("0_d.png", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("0_f.png", cv2.IMREAD_GRAYSCALE)
img3 = cv2.imread("0_dx.png", cv2.IMREAD_GRAYSCALE)
img4 = cv2.imread("0_fx.png", cv2.IMREAD_GRAYSCALE)

def a(img1, img2):
    # Redimensionar la segunda imagen al tamaño de la primera si no coinciden
    if img1.shape != img2.shape:
        img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]))

    # Restar las imágenes
    # cv2.subtract hace la resta y asegura que los valores negativos se corten a 0
    resultado = np.minimum(
        cv2.absdiff(img1.astype(np.uint8), img2.astype(np.uint8)),
        256 - cv2.absdiff(img1, img2),
        dtype=np.uint8
    )

    print(f"min: {resultado.min()}, max: {resultado.max()}, average: {np.average(resultado)}")
    #histograma:
    hist = cv2.calcHist([resultado], [0], None, [128], [0, 128])
    plt.plot(hist)
    return resultado

resultado = a(img1, img2)
resultado = a( cv2.imread("0_dx.png", cv2.IMREAD_GRAYSCALE), cv2.imread("0_fx.png", cv2.IMREAD_GRAYSCALE))
plt.show()

hist = cv2.calcHist([img1], [0], None, [256], [0, 256])
plt.plot(hist, label="double 1pt")
hist = cv2.calcHist([img2], [0], None, [256], [0, 256])
plt.plot(hist, label="float 1pt")
hist = cv2.calcHist([img3], [0], None, [256], [0, 256])
plt.plot(hist, label="double 100k pts")
hist = cv2.calcHist([img4], [0], None, [256], [0, 256])
plt.plot(hist, label="float 100k pts")
plt.legend()
plt.show()


cv2.imshow("A", img2)
cv2.imshow("Resta", resultado)
cv2.waitKey(0)
cv2.destroyAllWindows()
exit()
# Mostrar las imágenes
cv2.imshow("A", img2)

cv2.waitKey(0)
cv2.destroyAllWindows()