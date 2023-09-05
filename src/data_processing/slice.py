from PIL import Image

from src.util.file_dialog import select_file_from_dialog

CHIP_ROW_COUNT = 6
CHIP_COL_COUNT = 8

image_file = select_file_from_dialog(prompt="Select image to slice", allowed_extensions=["jpeg", "jpg", "png"])
image = Image.open(image_file)

# Get the dimensions of the image
width, height = image.size

# Calculate the width of each vertical slice and the height of each horizontal slice
vertical_slice_width = width // CHIP_COL_COUNT
horizontal_slice_height = height // CHIP_COL_COUNT

vertical_slices = []
horizontal_slices = []

# Iterate through the image and slice it into 8 vertical parts
for i in range(CHIP_COL_COUNT):
    left = i * vertical_slice_width
    right = (i + 1) * vertical_slice_width

    # Crop and append the vertical slice to the list
    vertical_slice_image = image.crop((left, 0, right, height))
    vertical_slices.append(vertical_slice_image)

# Iterate through the image and slice it into 6 horizontal parts
for i in range(CHIP_ROW_COUNT):
    top = i * horizontal_slice_height
    bottom = (i + 1) * horizontal_slice_height

    # Crop and append the horizontal slice to the list
    horizontal_slice_image = image.crop((0, top, width, bottom))
    horizontal_slices.append(horizontal_slice_image)

# Save or display the slices
for i, slice_image in enumerate(vertical_slices):
    slice_image.save(f"vertical_slice_{i+1}.png")
    # slice_image.show()
for i, slice_image in enumerate(horizontal_slices):
    slice_image.save(f"horizontal_slice_{i+1}.png")
    # slice_image.show()

print(f"Image sliced into {CHIP_COL_COUNT} vertical and {CHIP_ROW_COUNT} horizontal parts.")
