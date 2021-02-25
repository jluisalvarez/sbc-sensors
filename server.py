#!/usr/bin/env python3

from http.server import HTTPServer, BaseHTTPRequestHandler
from http import HTTPStatus
import multiprocessing as mp
import os
import json
import time
import psutil

from w1thermsensor import W1ThermSensor
# 1-wired
# PIN: RBPi=GPIO4    BBB=9_12
sensor = W1ThermSensor()

def readTemperature():
        temperature = sensor.get_temperature()
        return temperature


import adafruit_dht
# Library CircuitPython (Adafruit_CircuitPython_DHT)
# PIN: RBPi=GPIO23    BBB=??
pinHum = 23
dhtDevice = adafruit_dht.DHT11(pinHum)

def readHumidity():
    try:
        humidity = dhtDevice.humidity
        return humidity
    except RuntimeError:
         return -999.9

def read_data():


    t1 = time.perf_counter()
    temp = round(readTemperature(), 2)
    t2 = time.perf_counter()
    cpu_percents = []
    for iter in range(1):
        cpu_percents.append(monitor_cpu(target=readTemperature))
    Sum = sum(cpu_percents)
    total = len(cpu_percents)
    pcpuTem = round((Sum/total),2)
    tpoTemp = round((t2-t1),2)

    TempSensor = {
        "Temperat": temp,
        "pcpuTemperature": pcpuTem,
        "tpoTemperature": tpoTemp,
    }

    t1 = time.perf_counter()
    hum = round(readHumidity(), 2)
    t2 = time.perf_counter()
    cpu_percents = []
    for iter in range(1):
        cpu_percents.append(monitor_cpu(target=readHumidity))
    Sum = sum(cpu_percents)
    total = len(cpu_percents)
    pcpuHum = round((Sum/total),2)
    tpoHum = round((t2-t1),2)



    data = {
        "temperatureSensor": TempSensor,
        "Humidity": hum,
        "pcpuHumidity": pcpuHum,
        "tpoHumidity": tpoHum
    }
    return data


def monitor_cpu(target):
    worker_process = mp.Process(target=target)
    worker_process.start()
    p = psutil.Process(worker_process.pid)

    # log cpu usage of `worker_process` every 1 ms
    cpu_percents = []
    while worker_process.is_alive():
        cpu_percents.append(p.cpu_percent())
        time.sleep(0.001)
    worker_process.join()
    Sum = sum(cpu_percents)
    total = len(cpu_percents)
    pcpu = round((Sum/total)/100.0,5)
    return pcpu

class _RequestHandler(BaseHTTPRequestHandler):
    
    def _set_headers(self):
        self.send_response(HTTPStatus.OK.value)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()


    def do_GET(self):
        self._set_headers()
        data = read_data()
        response = json.dumps(data)
        response = bytes(response, 'utf-8')
        self.wfile.write(response)


def run_server():
    server_address = ('', 8001)
    httpd = HTTPServer(server_address, _RequestHandler)
    print('Start Sensoring Server at %s:%d' % server_address)
    print('Ctrl+C to end.')
    httpd.serve_forever()


if __name__ == '__main__':
    try:
        run_server()
    except KeyboardInterrupt:
        print ("Server closed.")
