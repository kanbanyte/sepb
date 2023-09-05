import subprocess
import importlib

def install_packages(packages_to_install, quiet_mode=True):
	"""
    Install packages.

    Args:
		packages_to_install (list[str]): list of package names to install
		quiet_mode (boolean): Whether all packages are installed in quiet mode or not
    Returns:
        sNone
    """
	for package in packages_to_install:
		if importlib.util.find_spec(package) is None:
			if quiet_mode:
				print(f"Installing {package} in quiet mode")
				subprocess.call(["pip", "install", package, "--quiet"])
			else:
				print(f"Installing {package}")
				subprocess.call(["pip", "install", package])
		else:
			print(f"{package} is already installed.")
