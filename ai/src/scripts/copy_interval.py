import os
import shutil
from util.file_dialog import select_folder_from_dialog

def main():
    src_dir = select_folder_from_dialog("Enter the name of the source folder: ")
    if not src_dir:
        raise ValueError(f"Source folder not selected")
    if not os.path.exists(src_dir):
        os.mkdir(src_dir)
    
    destination_dir = select_folder_from_dialog("Enter the name of the target folder: ")
    if not destination_dir:
        raise ValueError(f"Destination folder not selected")
    if not os.path.exists(destination_dir):
        os.mkdir(destination_dir)

    image_interval = int(input("Enter the interval to skip: "))
    if image_interval < 0:
        raise ValueError("Interval must be positive")

    files = os.listdir(src_dir)
    for i in range(0, len(files), image_interval):
        file_to_copy = os.path.join(src_dir, files[i])
        destination_path = os.path.join(destination_dir, files[i])
        shutil.copy(file_to_copy, destination_path)
        print(f"Copied: {file_to_copy} to {destination_path}")

    print("Copy complete.")

if __name__ == "__main__":
    main()
    