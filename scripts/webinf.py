#!/usr/bin/env python
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS, cross_origin
import os
import json
import subprocess as sp
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


if __name__ == '__main__':
   app.run(debug = True, host='0.0.0.0')