from PIL import Image
import numpy as np


def average_images(image_paths, output_path='average.png'):
    num = len(image_paths)

    # Load the first image and convert to numpy array
    img_array = np.array(Image.open(image_paths[0]), dtype=np.float64)

    # Accumulate pixel values
    for path in image_paths[1:]:
        img = Image.open(path)
        img_array += np.array(img, dtype=np.float64)

    # Average the pixel values
    img_array /= num
    img_array = np.clip(img_array, 0, 255).astype(np.uint8)

    # Save the averaged image
    averaged_image = Image.fromarray(img_array)
    averaged_image.save(output_path)
    print(f"Averaged image saved as: {output_path}")


# Example usage:
if __name__ == "__main__":
    # Replace with your actual image file paths
    image_files = [f"output/color/out/{i}.png" for i in range(0, 128)]

    average_images(image_files, output_path=f"output/color/average/average_{len(image_files)}.png")
