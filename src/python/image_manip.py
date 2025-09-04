import cv2
import numpy as np

# Cargar las dos imágenes
# Deben tener el mismo tamaño, de lo contrario habrá que redimensionarlas
img1 = cv2.imread("0_d.png", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("0_f.png", cv2.IMREAD_GRAYSCALE)

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

print(resultado.dtype, resultado.min(),  resultado.max(), np.average(resultado))

# Mostrar las imágenes
cv2.imshow("Resta", resultado)

cv2.waitKey(0)
cv2.destroyAllWindows()