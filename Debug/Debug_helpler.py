import math
from cv2 import transform
import numpy as np
from CommonLibs.UniversalLib import v2x_waypoint

def draw_v2x_waypint(ego_vehicle,map,debug_help):
    #waypoint_list = []
    waypoint_o = map.get_waypoint(ego_vehicle.get_transform().location)
    waypoints = waypoint_o.next(5) 
    #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
    for i in range(15):
        waypoint = waypoints[0]
        if waypoint.road_id>3 :
            delta_dis=0.5
        else:
            delta_dis =5
        location = waypoint.transform.location
        #waypoint_list.append(location)
        debug_help.draw_point(location,size=0.05,life_time=0.2)
        waypoints = waypoint.next(delta_dis)
        if waypoints==[]:
                #print(waypoints)
                break
def curv_calc_debug(transform,map,debug_help):
    '''
    calculate lane center curve.
    0.5 meter each point in the first 50 meter, and then 5 meters each point for the next 50 meter.
    '''
    waypoint_list=[]
    x=[]
    y=[]
    waypoint_o = map.get_waypoint(transform.location)
    waypoints = waypoint_o.next(0.5)
    for i in range(8):
        waypoint_o = waypoints[0]
        waypoints = waypoint_o.next(0.5)
        if waypoints==[]:
            #print(waypoints)
            break
    if  waypoint_o.is_junction:
        #print("juction: ",transform)
        f1=[0,0,0,0]
    else:
    #waypoint_list.append(waypoint_o)
        for i in range(100):
            if waypoints==[]:
            #print(waypoints)
                waypoints=waypoint_o.next(5)
            waypoint = waypoints[0]
            if waypoint.is_junction:
                break
            else:
                waypoint_list.append(waypoint)
                waypoints = waypoint.next(0.5)
            if waypoints==[]:
            #print(waypoints)
                break
        
        for i in range(10):
            if waypoints==[]:
            #print(waypoints)
                waypoints=waypoint_o.next(5)
            waypoint = waypoints[0]
            if waypoint.is_junction:
                break
            else:
                waypoint_list.append(waypoint)
                waypoints = waypoint.next(5)
            if waypoints==[]:
            #print(waypoints)S
                break
        for ii in waypoint_list:
            if abs(abs(math.sin(ii.transform.rotation.yaw/57.3))-abs(math.sin(transform.rotation.yaw/57.3)))<0.7:
                debug_help.draw_point(ii.transform.location,size=0.05,life_time=0.2)
                x0=ii.transform.location.x-transform.location.x
                y0=ii.transform.location.y-transform.location.y
                xx=x0*math.cos(transform.rotation.yaw/57.3)+y0*math.sin(transform.rotation.yaw/57.3)
                yy=-x0*math.sin(transform.rotation.yaw/57.3)+y0*math.cos(transform.rotation.yaw/57.3)
                x.append(xx)
                y.append(yy)
        x=np.array(x)
        y=np.array(y)
        if len(x)<1:
            f1=[0,0,0,0]
        else:
            f1=np.polyfit(x,y,3)#fit 3-d curve, return a list for curve parameters
          
    return f1

def log_waypoint(ego_vehicle,log_loc):
    transform=ego_vehicle.get_transform()
    print(transform.location,file=log_loc)