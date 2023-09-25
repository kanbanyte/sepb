# Directory Structure
The below list provides an overview of the directory structure.\
See the `README.md` files within individual subdirectories for more details.
* [camera](camera/README.md): contains modules interacting with the ZED SDK Python API.
* [data_processing](data_processing/README.md): contains modules and runnable scripts that process images and interact with the ZED2i camera.
* [models](models/python/README.md): contains modules related to a Python class representing a trained model.
* [samples](samples/python/README.md): contains sample programs that use trained models to detect objects.
* [training](training/README.md): contains notebooks and scripts that train a model as well as visualize its metrics.
* [util](util/README.md): utility functions, mostly related to the file system.

Directories that act as packages have an `__init__.py` file that imports all modules within that package.
Code using these packages will have access to all modules inside those packages, regardless of nesting level, without having to specify their absolute paths.

## Installation
* Note: use `python` if you are running in Windows and `python3` if you are running in Linux.
This document exclusively uses the Windows version.
* To ensure you have the latest pip, run the command:
```bash
python -m pip install --upgrade pip
```
* To install the package, navigate to the project containing `pyproject.toml` and run the command:
```bash
python -m pip install .
```

## Dependencies
### Python Packages
* To ensure that the machine has packages to run every script in this directory, run the following command:
```bash
python -m pip install -r ./requirements.txt
```

### ZED2 SDK
* Install the ZED2 SDK at [the official site](https://www.stereolabs.com/developers/release/).
* Install the Python API by running the `get_python_api.py` script:
* Install the Python API by running the `get_python_api.py` script:
	* On Windows, the script is in `C:\Program Files (x86)\ZED SDK\`
	* On Linux, the script is in `/usr/local/zed/`
	* See the [official documentation](https://www.stereolabs.com/docs/app-development/python/install/#installing-the-python-api) for more information.