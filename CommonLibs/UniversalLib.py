import carla
import math
import numpy as np

def get_boundingbox(vehicle):
    '''
    get bounding box of vehicle, carla API
    '''
    udp_list=[]
    veh_v = vehicle.bounding_box.extent#get bounding box
    udp_list.append(veh_v.x)
    udp_list.append(veh_v.y)
    udp_list.append(veh_v.z)
    return udp_list

def lane_color(color): 
    '''
    get lane color with following mapping
        white:1
        yellow:2
        other:0
    '''
    if(color == carla.LaneMarkingColor.White):
        return 1
    elif(color ==carla.LaneMarkingColor.Yellow):
        return 2
    else:
        return 0

def lane_mark(mark):
    '''
    get lane mark with following mapping
        Solid:1
        Broken:0
        SoilidSolid:1
        Others:0
    '''
    if(mark== carla.LaneMarkingType.Solid):
        return 1
    elif(mark ==carla.LaneMarkingType.Broken):
        return 0
    elif(mark== carla.LaneMarkingType.SolidSolid):
        return 1
    else:
        return 0


def curv_calc(transform,map):
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
                waypoints= waypoint_o.next(5)
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
                waypoints= waypoint_o.next(5)
            waypoint = waypoints[0]
            if waypoint.is_junction:
                break
            else:
                waypoint_list.append(waypoint)
                waypoints = waypoint.next(5)
            if waypoints==[]:
                #print(waypoints)
                break
        for ii in waypoint_list:
            if abs(abs(math.sin(ii.transform.rotation.yaw/57.3))-abs(math.sin(transform.rotation.yaw/57.3)))<0.7:
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
'''
def v2x_waypoint(transform,map):
    waypoint_list = []
    waypoint_o = map.get_waypoint(transform.location)
    waypoints = waypoint_o.next(5) 
    #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
    for i in range(15):
        waypoint = waypoints[0]
        location = waypoint.transform.location
        geoLocation=map.transform_to_geolocation(location)
        #print(geoLocation)
        waypoint_list.append(geoLocation.latitude)
        waypoint_list.append(geoLocation.longitude)
        waypoints = waypoint.next(5)
    #print("*********************************************************************")
    return waypoint_list
'''    
def v2x_waypoint(transform,map):
    waypoint_list = []
    waypoint_o = map.get_waypoint(transform.location)
    waypoints = waypoint_o.next(5) 
    #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
    for i in range(15):
        waypoint = waypoints[0]
        if waypoint.road_id>3 :
            delta_dis=0.5
        else:
            delta_dis =5
        location = waypoint.transform.location
        geoLocation=map.transform_to_geolocation(location)
        waypoint_list.append(geoLocation.latitude)
        waypoint_list.append(geoLocation.longitude)
        waypoints = waypoint.next(delta_dis)
        if waypoints==[]:
                #print(waypoints)
                break
    return waypoint_list