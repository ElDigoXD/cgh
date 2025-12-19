## Como utilizar los programas

> Los ficheros se encuentran en cmake-bulid-releases

### GUI (Interfaz gráfica para visualizar escenas)

Para obtener una imagen de una escena, se puede configurar desde la interfaz el numero de rebotes y de muestras por pixel. Una vez terminado el render, hay que hacer click en el botón "Save" para guardar la imagen (se guarda en la carpeta output con la fecha actual como nombre).

Para obtener un holograma, tambien se puede configurar el numero de rebotes y de muestras de cada punto, es necesario marcar "Enable render" y "Enable render cgh" (la opcion de oclusión estricta NO genera una nueva nube de puntos).  El numero de puntos se puede configurar en la pestaña "Scene" (NxM puntos). 

### PC (Visualizaaor de nubes de puntos y CLI para generar hologramas)

Al ejecutar el programa sin el argumento "--headless", se muestra la interfaz gráfica para visualizar nubes de puntos. Visualiza la nube de puntos dependiendo del argumento "--occ" (point_cloud_[mix|ortho].bin).

Para obtener hologramas desde la terminal, hay que ejecutar el programa con el argumento "--headless" y los argumentos necesarios para configurar el cálculo del holograma. Estos se pueden consultar mediante el argumento "--help":

```
./pc {OPTIONS}
OPTIONS:
  -h, --help                        Display this help menu
  -p, --points                      Target number of points
  --headless                        Headless mode
  --gpu                             Use GPU
  --cpu                             Use CPU
  --color                           Use color
  --gs, --grayscale                 Use grayscale
  -o, --occ                         Use strict occlusion
  --num, --num-images               Number of images
  --gen, --generate_pc              Generate point cloud (ortho, mix, mesh)
  --scene                           Scene to use
```

Para modificar el numero de rebotes y de muestras por punto (solamente al generar la nube de puntos) es necesario modificar el cóodigo fuente en main_pc.cpp (linea 158-164: Renderer renderer{}).

### Modificar el pixel pitch

Para modificar el pixel pitch, es necesario editar el codigo fuente. Para el cálculo del CGH, hay que modificar la macro VIRTUAL_SLM_FACTOR en el archivo config.h (1 = 8um, 2 = 4um, 4 = 2um, 8 = 1um). Para la propagación, hay que modificar la el parámetro de la función propagate en propagate.py (linea 157).

## Scripts de python

### Propagate.py

Los argumentos son:

```
usage: propagate.py [-h] [-g] [-p] [-c COUNT] [-rgb] [-z Z] CGH

positional arguments:
  CGH                   Path to the CGH file in PNG, CSV, MAT, or BIN format

options:
  -h, --help            show this help message and exit
  -g, --grayscale       Grayscale image
  -p, --phase_only      Phase only image
  -c COUNT, --count COUNT
                        Number of images to propagate
  -z Z, --z Z           Z distance to propagate
```

Los archivos de salida se guardan en la carpeta output/propagation/color, en el caso de un solo holograma en color. En el caso de varios hologramas (COUNT > 1) se guardan en la carpeta out, dentro de la ruta de entrada.

Algunos ejemplos:

```bash
# Propaga un holograma en color a 296mm
python3 src/python/propagate.py out/gray_bg/0.png --z 296
# Propaga 10 hologramas en color a 296mm con el nombre {0..9}.png
python3 src/python/propagate.py out/gray_bg/ --z 296 -c 10
# Propaga un holograma en escala de grises (utilizando el 4º canal, o el único)
python3 src/python/propagate.py out/gray_bg/0.png -g --z 296
```

### Calibrate.py

Calibra todos los CGHs de una carpeta para el SLM.

Los argumentos son:

```
positional arguments:
  CGH                   Path to the CGH directory
options:
  -c COUNT, --count COUNT
                        Number of images to calibrate
```

> Los archivos de entrada deben llamarse {0..COUNT}.png.
> Los archivos de salida se guardan en la carpeta calibrado.

Algunos ejemplos:

```bash
# Calibra todos los CGHs de la carpeta out/gray_bg/
python3 src/python/calibrate.py out/gray_bg/ -c 10
# Calibra el CGH 0.png de la carpeta out/gray_bg/
python3 src/python/calibrate.py out/gray_bg/
```

## Comandos de ejemplo:

```bash
# Preparar el proyecto si no existen los archivos de build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=ninja -G Ninja -S . -B cmake-build-release
# Compilar el proyecto (target gui o pc)
cmake --build cmake-build-release --target pc -j 14
```

> Los comandos python3 se ejecutan desde la carpeta raíz.
>
> Los comandos ./pc se ejecutan desde la carpeta cmake-build-release.

```bash
# Genera una nube de puntos de la escena predefinida
./pc --headless --gen ortho
# Calcula un holograma con gpu, color, oclusion sencilla y 1000000 puntos 
./pc --headless --gpu --color --points 1000000
# Propaga un holograma a 293mm
python3 src/python/propagate.py 0.png  --z 293
# Propaga un holograma en un rango
# (hay que modificar el script (propagate_range))
python3 src/python/propagate.py 0.png
# Crea un mp4 con todas las propagaciones (desde la carpeta con los archivos)
ffmpeg -framerate 12 -pattern_type glob -i "*.png" -c:v libx264 -pix_fmt yuv420p output.mp4

# Igual que el anterior pero 10 hologramas con todos los puntos
./pc --headless --gpu --color --points 0 --num 10
```

Para generar una nube de puntos a partir de una escena:

```bash
./pc --headless --gen ortho --scene <nombre_de_la_escena>
```

Para generar un holograma a partir de una nube de puntos:

```bash
# Con GPU y color
./pc --headless --gpu --color
# Con CPU, escala de grises y oclusion estricta
# (utiliza la nube de puntos "point_cloud_mix.bin")
./pc --headless --cpu --grayscale --occ
```

## Archivos

Carpetas:

- resources: modelos, materiales, fuentes, etc.
- src: fuentes del programa y scripts de python
	- python: scripts de python

Archivos:

- CMakeLists.txt: Archivo del sistema de build CMake
- Makefile: Otro archivo para el sistema de build

src/python:

- all_in_one.py: Nada
- average.py: Nada
- calibrate.py: Calibra un holograma, sea color o gris para el slm
- cc.py: Calcula el coeficiente de correlación
- compare.py: Nada
- image_manip.py: Resta dos matrices de fase
- point_cloud.py: Nada
- propagate.py: Propaga hologramas

src:

- AABB.h: Axis Aligned Bounding Box e Intervalos
- BRDF2s.h: Función de distribución de reflectancia bidireccional (materiales)
- BRDFs.h: Interfáz de material
- Color.h: Clase que representa un color
- config.h: Configuración mediante defines (durante compilación)
- cuda.cu: Contiene codigo especifico para gpu (geometría y kernels)
- cuda.h: Header de cuda.cu
- GGXBRDF.h: Material GGX
- main.cpp: Ejecutable con la interfaz principal
- main_pc.cpp: Ejecutable con la interfaz de visualizar la nube de puntos y CLI (Command Line Interface) para generar hologramas
- Material.h: Otra abstracción sobre BRDF
- Mesh.h: Representa una malla. Tiene operaciones para modificarla y generar el BVH
- ObjReader.h: Lee archivos .obj
- OrthoCamera.h: Representa una cámara ortográfica
- PointCloud.h: Representa una nube de puntos y sus operaciones
- Random.h: Para generar numeros aleatorios
- Ray.h: Representa un rayo y sus operaciones
- Renderer.h: Se encarga de toda la funcionalidad de obtener imagenes a partir de una escena (CGI, CGH, calcular el color, etc.)
- Scene.h: Representa una escena
- Scenes.h: Definiciones de escenas en código
- Triangle.h: Todo lo relacionado con triángulos (Triangle, TriangleIntersection, Face)
- typedefs.h: Definiciones de tipos
- utils.h: Funciones de utilidad variadas
- Vecf.h: Vector float
- Vector.h: Vector double

> Los archivos que se utilizan en CGHs son Renderer, cuda y main_pc.
