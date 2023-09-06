import yaml

def read_yaml(config_file):
    with open(config_file, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
        return yaml_data