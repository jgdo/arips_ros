#!/usr/bin/env python

from flask import Flask, request, send_from_directory
import flask.json
import threading
import requests
import os
import rospkg

app = Flask(__name__)

rospack = rospkg.RosPack()
html_base_path = os.path.join(rospack.get_path('arips_web_dashboard'), 'html')
assert os.path.isfile(os.path.join(html_base_path, 'dashboard.html'))
assert os.path.isfile(os.path.join(html_base_path, 'dashboard_style.css'))
assert os.path.isfile(os.path.join(html_base_path, 'roslib.min.js'))

@app.route('/')
def dashboard():    
    return send_from_directory(html_base_path, 'dashboard.html')

@app.route('/dashboard_style.css')
def style():
    return send_from_directory(html_base_path, 'dashboard_style.css')

@app.route('/roslib.js')
def roslib():
    return send_from_directory(html_base_path, 'roslib.min.js')

def main():
    app.run(host="0.0.0.0", port=8080)


if __name__ == '__main__':
    main()

