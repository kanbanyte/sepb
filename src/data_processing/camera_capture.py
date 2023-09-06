import sys
import cv2
import os
import time
import pyzed.sl as sl

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../util"))
from file_reader import read_yaml
from cli_runner import install_packages

def open_camera(config):
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

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    camera = sl.Camera()   
    open_result = camera.open(init_params) 
    if  open_result != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open camera: {open_result}")
        exit -1

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

def take_photo(camera, crop_box):
    x1, x2, y1, y2 = crop_box
    image = sl.Mat()
    err = camera.grab()
    if err == sl.ERROR_CODE.SUCCESS:
        camera.retrieve_image(image, sl.VIEW.LEFT)
        # cv2.imshow("ZED", image.get_data())

        apply_crop(x1,x2,y1,y2,image)
        camera.close()
    else:
        print(f"Error: {err}")
        exit -1

def apply_crop(x1, y1, x2, y2, image):
    cropped_image = image[y1:y2, x1:x2]
    cv2.imshow("Cropped image", cropped_image)

def read_crop_box(config):
    crop_box = config.get('chip_slot_crop_box').get('left')
    x1 = crop_box.get('x1')
    x2 = crop_box.get('y1')
    y1 = crop_box.get('x2')
    y2 = crop_box.get('y2')
    return x1, x2, y1, y2

def main():
    # install_packages(["opencv-python", "cython", "numpy", "pyopengl", "yaml"])
    current_directory = os.path.dirname(os.path.realpath(__file__))
    file_name = "camera_config.yaml"
    file_path = os.path.join(current_directory, file_name)
    config = read_yaml(file_path)
    
    crop_box = read_crop_box(config)
    camera = open_camera(config)
    
    start_time = time.perf_counter()
    take_photo(camera, crop_box)
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time:.4f} seconds")

if __name__ == "__main__":
    main()
