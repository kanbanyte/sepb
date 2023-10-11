import yaml

def read_yaml(file_path):
	'''
	Read the content of a YAML file.

	Args:
		file_path (str): path to the YAML file.

	Returns:
		dict | list: content of the YAML file
	'''

	if not file_path.endswith(".yaml"):
		raise ValueError("Invalid YAML file extension")

	with open(file_path, 'r') as yaml_file:
		yaml_data = yaml.safe_load(yaml_file)
		return yaml_data
