import socket
import struct
import math
import carla

from CommonLibs.UniversalLib import get_boundingbox,lane_color,lane_mark,curv_calc,v2x_waypoint
from Localization.LocalizationManager import vehicle_loc


class UdpCtrl:
    '''
    Parameter
    ---------------------
    init_setting: object, including setup from ini file
    map: carla.Map
    ego_vehicle_object: EgoVehicle object
    vehicles: list, vehicle list
    semantic_lidar_object: SemanticLidarSensor object
    traffic_light_object: TFL object
    '''
    def __init__(self,init_setting):
        self.BUFSIZE = 1500
        self.init_setting=init_setting
        #udp server
        ip_port_s = (init_setting.rx_ip, 9999)
        self.udp_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
        self.udp_receiver.bind(ip_port_s)
        #udp client
        self.ip_port_c = (self.init_setting.tx_ip, 9998)
        self.udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        #udp client for V2X gps
        self.ip_port_v2x = (self.init_setting.tx_ip, 9997)
        self.udp_sender_v2x = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        self.received_data=()
    def msg_sender(self,map,ego_vehicle_object,vehicles,semantic_lidar_object,traffic_light_object,init_setting):
        udp_list=[]
        #ego vehicle packet
        for ii in ego_vehicle_object.get_roadinfo(map):
            udp_list.append(ii)
        #target vehicle packet
        for ii in semantic_lidar_object.get_nearby(vehicles,map):
                udp_list.append(ii)        
        if (len(udp_list)<352):
            for ii in range(352-len(udp_list)):
                udp_list.append(0)  
        for ii in self.get_traffic_info(vehicles[0],map,traffic_light_object,init_setting):
                udp_list.append(ii)
        self.udp_sender.sendto(struct.pack("{}d".format(len(udp_list)),*udp_list),self.ip_port_c)
    def msg_sender_v2x(self,ego_vehicle,map):
        udp_list=[]
        transform = ego_vehicle.get_transform()
        udp_list = v2x_waypoint(transform,map)
        self.udp_sender_v2x.sendto(struct.pack("{}f".format(len(udp_list)),*udp_list),self.ip_port_v2x)

    def msg_receiver(self):
        data,client_addr = self.udp_receiver.recvfrom(self.BUFSIZE)
        self.received_data=struct.unpack("13d",data)
        return self.received_data

    def destroy(self):
        self.udp_receiver.close()
        self.udp_sender.close()
        self.udp_sender_v2x.close()

    
    def get_traffic_info(self,ego_vehicle,map,traffic_light_object,init_setting): #get traffic light info 
        udp_list=[]
        transform = ego_vehicle.get_transform()
        waypoint= map.get_waypoint(transform.location)
        V2X_mode=init_setting.v2x_switch
        if V2X_mode=='on':
            for ii in traffic_light_object.get_traffic_light(self.received_data):
                udp_list.append(ii)
            '''
            #assume stop line in y=20
            if transform.location.y-20-2.5>0:
                stop_line_d=(transform.location.y-20-2.5)
            else:
                stop_line_d=0
            '''
            if waypoint.road_id == 3 or waypoint.road_id==1:
                stop_line_d=waypoint.s
            elif waypoint.road_id == 2 :
                stop_line_d=585.0-waypoint.s
            elif waypoint.road_id ==0:
                stop_line_d =476.0-waypoint.s
            else:
                stop_line_d=0
            udp_list.append(abs(stop_line_d))

        else:
            udp_list.append(3)
            udp_list.append(0)
            udp_list.append(0)
            udp_list.append(0)

        for i in range(9):
            udp_list.append(0)
        geolocation=map.transform_to_geolocation(transform.location)
        multi = 2**32
        latitude=multi*geolocation.latitude/360
        longitude=multi*geolocation.longitude/360
        udp_list.append(latitude)
        udp_list.append(longitude)
        for ii in curv_calc(transform,map):
            udp_list.append(ii)
        for ii in get_boundingbox(ego_vehicle):#get ego vehicle bouding box
            udp_list.append(ii)
        return udp_list