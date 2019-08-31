import packetserial_slip
import struct
import csv
import numpy
import time

#import threading

'''
  Stressmonster pressure recording file

  collects data, calculates stress_level and stores the results in a csv-file.

  csv-file used by node-red

  file edited by :Johannes Maier
  mail:ton.maier@gmail.com
'''

# Every 500 ms some data currently. This can be changed through the Source-code of the MCU
# Remember the UART-Bit Rate

# Values are calculated and stored in csv-file

get_pressure_packages = packetserial_slip.get_packages
pressure_data = []
got_atmospheric_pressure = False
atmospheric_pressure = 0.0
pressure_ball = 0.0
previous_p_ball= 0.0

def bytes_to_samples(byte_data, sample_width = 4, fmt = 'i'):
    if len(byte_data) % sample_width == 0 :
        samples_length = str(len(byte_data)//sample_width)
        return struct.unpack('<'+ samples_length + fmt,byte_data)
    else :
        return False

# Calculates mean pressure
def mean_pressure(pressure_data):
    mean_values =[]
    for sample in pressure_data:
        mean_values.append(numpy.mean(sample))
    return numpy.mean(mean_values)

# Calculates Stress-Level depending on the atmospheric pressure and the pressure inside the ball
def calc_stress(pressure_ball,atmospheric_pressure):
    stress_level = (pressure_ball - atmospheric_pressure)*(100/500) # random const value
    return stress_level

with  open('/home/pi/stressmonster_ws/src/pressure.csv', 'w', newline='') as csvfile:
    fieldnames = ['pressure_mean_value', 'pressure_atmosphere', 'stress_level']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    while not got_atmospheric_pressure:
        data_packages = get_pressure_packages()
        if data_packages:
            for byte_package in data_packages:
                samples = bytes_to_samples(byte_package)
                if samples:
                    pressure_data.append(samples)
                    if len(pressure_data) >=10 :
                        atmospheric_pressure = mean_pressure(pressure_data)
                        got_atmospheric_pressure = True
                        pressure_data.clear()
        else : continue

    while True:
        data_packages = get_pressure_packages()
        if data_packages:
            for byte_package in data_packages:
                samples = bytes_to_samples(byte_package)
                if samples:
                    pressure_data.append(samples)
                    if len(pressure_data) >=10:
                        pressure_ball = mean_pressure(pressure_data)

                        if pressure_ball > (atmospheric_pressure + 10) or previous_p_ball > (atmospheric_pressure + 10) :
                            if pressure_ball < previous_p_ball :
                                pressure_ball = previous_p_ball
                                pressure_ball -= 10
                                previous_p_ball = pressure_ball
                            else:
                                previous_p_ball = pressure_ball    

                        else: 
                            pressure_ball = atmospheric_pressure
                            previous_p_ball = atmospheric_pressure

                        stress_level = calc_stress(pressure_ball,atmospheric_pressure)
                        writer.writeheader()
                       
                        #Namensgebung nicht ganz korrekt, da durch die If-Schleife vorher ein Filter niedrigere Werte als den Atmosphärendruck nicht abspeichert
                        writer.writerow({'pressure_mean_value': int(pressure_ball),'pressure_atmosphere': int(atmospheric_pressure),'stress_level':int(stress_level)})
                        csvfile.seek(0,0)
                        pressure_data.clear()
        else : continue
        time.sleep(0.5)    # vorlaeufige Methode

