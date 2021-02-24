#!/usr/bin/env python
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS, cross_origin
import os
import json
import subprocess as sp
import base64
from PIL import Image
import io
from io import BytesIO
import yaml
import socket

app = Flask(__name__)

CORS(app)

procA = None

@app.route("/")
def hello():
    return "Hello World!!!!"

@app.route('/uploadXML', methods = ['GET', 'POST'])
def upload_file():
   
   if request.method == 'POST':
      data = request.get_json()
      
      print(data)
      file1 = open("/home/pi/nginx/agv_html_interface/public/"+data["filename"] + ".xml","w+") 
      file1.write(data["data"])
      return 'file uploaded successfully'
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))

@app.route('/run', methods = ['GET', 'POST'])
def run_file():
   
   if request.method == 'POST':
      global procA
      
      data = request.get_json()
      
      file1 = open("/home/pi/nginx/agv_html_interface/public/test1.py","w+") 
      file1.write(data["data"])
      file1.close()
      procA = sp.Popen(['python', '/home/pi/nginx/agv_html_interface/public/test1.py'])

      print(data["data"])

      
      return 'file uploaded successfully'
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))

@app.route('/stop', methods = ['GET', 'POST'])
def stop_file():
   if request.method == 'POST':
      global procA
      procA.terminate()
      procA = None
      return 'file uploaded successfully'
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))


@app.route('/getXML', methods = ['GET', 'POST'])
def get_file():
   
   if request.method == 'GET':
      rr = []
      for file in os.listdir("/home/pi/nginx/agv_html_interface/public"):
         if file.endswith(".xml"):
            print(file)
            data = {}
            data['name'] = file
            rr.append(data)

      return jsonify(files=rr)
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))


@app.route('/getXMLdata', methods = ['GET', 'POST'])
def get_data():
   
   if request.method == 'POST':
      data = request.get_json()

      f = open("/home/pi/nginx/agv_html_interface/public/"+data["filename"], "r")
     
      data1 = {}
      data1['content'] = f.read()
      return data1
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))


@app.route('/getMap', methods = ['GET', 'POST'])
def getMap():
   
   if request.method == 'POST':
      data = request.get_json()
      im = Image.open("/home/pi/linorobot_ws/src/linorobot/maps/" + data["mapname"])
      file_object = io.BytesIO()
      im.save(file_object, 'PNG')

      #with open("/home/pi/linorobot_ws/src/linorobot/maps/" + data["mapname"], "rb") as imageFile:
      #   encoded_img = base64.b64encode(imageFile.read())

      file_object.seek(0)

      encoded_img = base64.b64encode(file_object.getvalue()).decode('ascii')
      res =  { 'Status' : 'Success', 'ImageBytes': encoded_img}

      return jsonify(res), 201
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))

@app.route('/config', methods = ['GET', 'POST'])
def config():
   if request.method == 'POST':
      data = request.get_json()
      with open(r'/home/pi/linorobot_ws/src/mqtt_bridge/config/demo_params.yaml') as file:
         documents = yaml.full_load(file)
         items = documents.items()

         for item, doc in documents.items():
            print(item, ":", doc)
      print("-------------------")
      print(documents['mqtt']['connection']['host'])

      hostname = socket.gethostname()

      documents['mqtt']['connection']['host'] = data['central_ip']
      documents['bridge'][0]['topic_from'] = '/' + hostname + '/visualization_marker'
      documents['bridge'][0]['topic_to'] = '/'/ + hostname + '/visualization_marker'

      with open(r'/home/pi/linorobot_ws/src/mqtt_bridge/config/demo_params.yaml', 'w') as file:
         doc = yaml.dump(documents, file)

      res =  { 'Status' : 'Success'}

      return jsonify(res), 201
   else:
      raise RuntimeError("Weird - don't know how to handle method {}".format(request.method))

   

if __name__ == '__main__':
   app.run(debug = True, host='0.0.0.0')
