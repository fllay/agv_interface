#!/usr/bin/env python3
import yaml
import socket
with open(r'/home/pi/linorobot_ws/src/mqtt_bridge/config/demo_params.yaml') as file:
    documents = yaml.full_load(file)
    items = documents.items()

    for item, doc in documents.items():
        print(item, ":", doc)
    print("-------------------")
    print(documents['mqtt']['connection']['host'])

hostname = socket.gethostname()

documents['mqtt']['connection']['host'] = '192.168.68.107'
documents['bridge'][0]['topic_from'] = '/' + hostname + '/visualization_marker'
documents['bridge'][0]['topic_to'] = '/' + hostname + '/visualization_marker'

with open(r'/home/pi/linorobot_ws/src/mqtt_bridge/config/demo_params.yaml', 'w') as file:
    doc = yaml.dump(documents, file)
