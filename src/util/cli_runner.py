import subprocess
import importlib

def install_packages(packages_to_install, quiet_mode=True, args = None):
	"""
    Install packages.

    Args:
		packages_to_install (list[str]): list of package names to install
		quiet_mode (boolean): Whether all packages are installed in quiet mode or not
    Returns:
        None
    """
	for package in packages_to_install:
		if importlib.util.find_spec(package) is None:
			if quiet_mode:
				print(f"Installing {package} in quiet mode")
				run_command(f"pip install {package} --quiet {args}")
			else:
				print(f"Installing {package}")
				run_command(f"pip install {package} {args}")
		else:
			print(f"{package} is already installed.")

def run_command(command):
	subprocess.call(command.split())
