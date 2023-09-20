# Overview of the Vision System source code

## Dependencies

### Python packages

* To ensure that the machine has packages to run every script in this directory, run the following command:
```bash
python -m pip install -r ./requirements.txt
```
### ZED2 SDK

* Install the ZED2 SDK at [the official site](https://www.stereolabs.com/developers/release/)
* Install the Python API by running the `get_python_api.py` script:
    * On Windows, the script is in `C:\Program Files (x86)\ZED SDK\`
    * On Linux, the script is in `/usr/local/zed/`
    * See the [official documentation](https://www.stereolabs.com/docs/app-development/python/install/#installing-the-python-api) for more information.

## Directory structure

The below list provides an overview of the directory structure.
See the README.md files within individual subdirectories for more details.
* `data_processing`: contains modules and runnable scripts that process images and interact with the ZED2i camera
* `models`: contains modules related to a Python class representing a trained model.
* `samples`: contains sample progra
* `training`: contains notebooks and scripts that train the model as well as visualize the model metrics.ms that run inference with a trained model.
* `util`: utility functions, mostly related to the file system.