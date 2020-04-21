import json


data = [(1, 2), (2, 3)]
with open('nodes.json', 'w') as file:
    json.dump(data, file)
