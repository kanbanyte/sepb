import sys
import cv2
import yaml
import os
import time
import pyzed.sl as sl
sys.path.append("../util")

from cli_runner import install_packages

def get_camera(config):
    camera_properties = config.get('camera', {})
    brightness = camera_properties.get('brightness', None)
    contrast = camera_properties.get('contrast', None)
    hue = camera_properties.get('hue', None)
    saturation = camera_properties.get('saturation', None)
    sharpness = camera_properties.get('sharpness', None)
    gamma = camera_properties.get('gamma', None)
    white_balance = camera_properties.get('white_balance', None)
    gain = camera_properties.get('gain', None)
    exposure = camera_properties.get('exposure', None)

    camera = sl.Camera()

    camera.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, contrast)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, saturation)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, brightness)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.HUE, hue)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, sharpness)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, gamma)

    # apply default
    camera.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, exposure)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, gain)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, white_balance)

    return camera

def read_yaml(config_file):
    with open(config_file, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
        return yaml_data

def take_photo(config):
    crop_box = config.get('chip_slot_crop_box', {})
    camera = get_camera(config)
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    camera.open(init_params)
    image = sl.Mat()
    err = camera.grab()
    if err == sl.ERROR_CODE.SUCCESS:
        camera.retrieve_image(image, sl.VIEW.LEFT)
        # cv2.imshow("ZED", image.get_data())
        x1 = crop_box.get('x1', None)
        x2 = crop_box.get('y1', None)
        y1 = crop_box.get('x2', None)
        y2 = crop_box.get('y2', None)

        apply_crop(x1,x2,y1,y2,image)
        camera.close()
    else:
        print(f"Error: {err}")
        exit -1

def apply_crop(x1, y1, x2, y2, image):
    cropped_image = image[y1:y2, x1:x2]
    cv2.imshow("Cropped image", cropped_image)

def main():
    # install_packages(["opencv-python", "cython", "numpy", "pyopengl"])
    current_directory = os.getcwd()
    file_name = "camera_config.yaml"
    file_path = os.path.join(current_directory, "src/data_processing", file_name)
    config = read_yaml(file_path)
    start_time = time.perf_counter()
    take_photo(config)
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time:.4f} seconds")


if __name__ == "__main__":
    main()
