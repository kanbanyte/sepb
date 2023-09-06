import subprocess
import importlib

def install_packages(packages_to_install, quiet_mode=True, args = None):
	"""
    Install packages using the command `python -m pip install <package name>.
	The shorthand `pip install <package name>` command is not used to
	avoid mismatch between the active Python interpreter and package manager.

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
				run_command(f"python -m pip install {package} --quiet {args}")
			else:
				print(f"Installing {package}")
				run_command(f"python -m pip install {package} {args}")
		else:
			print(f"{package} is already installed.")

def run_command(command):
	"""
	Run a CLI command and stops the script if the command fails.

	Args:
		command (str): command to run

	Returns:
		None
	"""
	print(f"Running command `{command}`")
	exit_code = subprocess.call(command.split())
	if exit_code is not 0:
		print(f"The command `{command}` failed. Exiting")
		exit(exit_code)
