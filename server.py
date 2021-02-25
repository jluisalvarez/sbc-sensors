#!/usr/bin/env python3

from http.server import HTTPServer, BaseHTTPRequestHandler
from http import HTTPStatus
import multiprocessing as mp
import os
import json
import time
import psutil
import RPi.GPIO as GPIO

# Ultrasonic Sensor HC-SR04
# GPIO 
# PIN: RBPi --> TRIG=25, ECHO=24     BBB -->
def readDistance():
    TRIG = 25 
    ECHO = 24 

    GPIO.setmode(GPIO.BCM)                
    GPIO.setup(TRIG, GPIO.OUT)  
    GPIO.setup(ECHO, GPIO.IN)

    # Ponemos en bajo el pin TRIG y después esperamos 0.5 seg para que el transductor se estabilice
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.5)

    #Ponemos en alto el pin TRIG esperamos 10 uS antes de ponerlo en bajo
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)

    # En este momento el sensor envía 8 pulsos ultrasónicos de 40kHz y coloca su pin ECHO en alto
    # Debemos detectar dicho evento para iniciar la medición del tiempo
        
    while True:
        pulso_inicio = time.time()
        if GPIO.input(ECHO) == GPIO.HIGH:
            break

    # El pin ECHO se mantendrá en HIGH hasta recibir el eco rebotado por el obstáculo. 
    # En ese momento el sensor pondrá el pin ECHO en bajo.

	# Prodedemos a detectar dicho evento para terminar la medición del tiempo
        
    while True:
        pulso_fin = time.time()
        if GPIO.input(ECHO) == GPIO.LOW:
            break

    # Tiempo medido en segundos
    duracion = pulso_fin - pulso_inicio

    #Obtenemos la distancia considerando que la señal recorre dos veces la distancia a medir y que la velocidad del sonido es 343m/s
    distancia = (34300 * duracion) / 2

    return distancia



# Temperature Sensor DS18B20
from w1thermsensor import W1ThermSensor
# 1-wired
# PIN: RBPi --> default Pin GPIO4    BBB --> default Pin 9_12
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
        "value": temp,
        "cpu": pcpuTem,
        "time": tpoTemp,
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

    HumSensor = {
        "value": hum,
        "cpu": pcpuHum,
        "time": tpoHum
    }

    t1 = time.perf_counter()
    dist = round(readDistance(), 2)
    t2 = time.perf_counter()
    cpu_percents = []
    for iter in range(1):
        cpu_percents.append(monitor_cpu(target=readDistance()))
    Sum = sum(cpu_percents)
    total = len(cpu_percents)
    pcpuDist = round((Sum/total),2)
    tpoDist = round((t2-t1),2)

    DistSensor = {
        "value": dist,
        "cpu": pcpuDist,
        "time": tpoDist
    }


    data = {
        "temperatureSensor": TempSensor,
        "humiditySensor": HumSensor,
        "UltrasocinSensor": DistSensor,
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
