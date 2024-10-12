import sys
import os
import time
import math
import numpy as np
from pyproj import Proj, transform
import simplekml


# access to the drivers 
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2')) 

import gps_driver_v2 as gpddrv

import imu9_driver_v2 as imudrv

import arduino_driver_v2 as arddrv

imu = imudrv.Imu9IO() # create an ARduino object


ard = arddrv.ArduinoIO()

left_speed_default = 100
right_speed_default = 100

projDegree2Meter = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
# reference point lake
reference_lat = 48.199111
reference_lon = -3.014930

desired_lat = 48.199659
desired_lon = -3.016061

reference_x,reference_y = projDegree2Meter(reference_lon,reference_lat)
print ("ref",reference_x,reference_y,reference_lon,reference_lat)

kml = simplekml.Kml()

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi 

def setThruster(left, right):
    ard.send_arduino_cmd_motor(left,right)

def cvt_gll_ddmm_2_dd (st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat/100))
    olon = float(int(ilon/100))
    olat_mm = (ilat%100)/60
    olon_mm = (ilon%100)/60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat,olon

gps = gpddrv.GpsIO() # create a GPS object
gps.set_filter_speed("0") # allowing GPS measures to change even if the DDBoat is not moving 


trajectory = []

while True:
    gll_ok,gll_data=gps.read_gll_non_blocking() 
    if gll_ok: # GPGLL message received
        
        lat,lon = cvt_gll_ddmm_2_dd(gll_data) # convert DDMM.MMMM to DD.DDDDD
        pnt = kml.newpoint(name="GPS", coords=[(lon,lat)])
        trajectory.append((lon, lat))
        x,y = projDegree2Meter(lon, lat) # convert to meters
        lat_check,lon_check = projDegree2Meter(x,y,inverse=True) # check conversion OK 
        dx = x - reference_x
        dy = y - reference_y
        # d = (dx**2 + dy**2)**0.5
        distance = np.sqrt(dx*dx+dy*dy)
        heading = 90 - np.degrees(np.arctan2(dy,dx))
        # get the magnetic raw data (raw = not calibrated !) 
        xmag,ymag,zmag = imu.read_mag_raw()

        # print ("lat=%.4f lon=%.4f (check %.4f %.4f) x=%.2f y=%.2f dx=%.2f, dy=%.2f, distance=%.2f, heading=%.2f"% (lat,lon,lat_check,lon_check,x,y,dx,dy,distance,heading))

        x1,y1 = projDegree2Meter(desired_lon, desired_lat)
        dx1 = x1 - reference_x
        dy1 = y1 - reference_y
        heading_desired = 90 - np.degrees(np.arctan2(dy1,dx1))

        # Compute the desired movement
        d = np.sqrt((x1 - x)**2 + (y1 - y)**2)
        w = sawtooth(heading_desired - heading) 
        
        #Control Law
        k_linear = (distance/100)*5
        k_angular = 2
        linear_velocity = k_linear * d
        angular_velocity = k_angular * w
        print("distance=%.4f linear=%.4f angular=%.4f lon=%.4f lat=%.4f"%(d, k_linear, k_angular, lon, lat))
        #right_thruster - left_thruster = 2K*w

        left_thruster = linear_velocity - angular_velocity
        right_thruster = linear_velocity + angular_velocity
        

        #margin to stop once close to the target point and save the path
        if(d<5):
            print("arrived")
            kml.save("gps_data.kml")
            break

        setThruster(left_thruster, right_thruster)

    



