import yaml
#import yaml
from yaml.loader import SafeLoader

with open('task_sequence.yaml') as f:
    data = yaml.load(f, Loader=SafeLoader)
    print(data)