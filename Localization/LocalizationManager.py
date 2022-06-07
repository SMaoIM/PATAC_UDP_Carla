import math
import numpy as np

def vehicle_loc(map,vehicle):
        udp_list=[]
        #get spped,acceleration,angular speed, position of  ego vehicle
        v_veh =np.mat([[vehicle.get_velocity().x*3.6],[vehicle.get_velocity().y*3.6],[vehicle.get_velocity().z*3.6]])
        a_veh =np.mat([[vehicle.get_acceleration().x],[vehicle.get_acceleration().y],[vehicle.get_acceleration().z]])
        aa_veh = np.mat([[vehicle.get_angular_velocity().y/57.3],[vehicle.get_angular_velocity().z/57.3],[vehicle.get_angular_velocity().x/57.3]])
        transform_o = vehicle.get_transform()    #vehicle transform in world coodination
        x_veh=np.mat([[transform_o.location.x],[transform_o.location.y],[transform_o.location.z]])
        x_a_veh=np.mat([[transform_o.rotation.pitch/57.3],[transform_o.rotation.yaw/57.3],[transform_o.rotation.roll/57.3]])
        waypoint_o = map.get_waypoint(transform_o.location)
        
        #set coordination-system transform matrix
        pitch_mat =np.mat([[math.cos(x_a_veh[0]),0,math.sin(x_a_veh[0])],[0,1,0],[-math.sin(x_a_veh[0]),0,math.cos(x_a_veh[0])]])
        yaw_mat = np.mat([[math.cos(x_a_veh[1]),math.sin(x_a_veh[1]),0],[-math.sin(x_a_veh[1]),math.cos(x_a_veh[1]),0],[0,0,1]])
        roll_mat = np.mat([[1,0,0],[0,math.cos(x_a_veh[2]),-math.sin(x_a_veh[2])],[0,math.sin(x_a_veh[2]),math.cos(x_a_veh[2])]])
        trans_mat =np.dot(pitch_mat,yaw_mat,roll_mat) 
        #calculate vehicle info in vehicle coordination sysytem
        v_out =np.dot(trans_mat,v_veh)
        a_out =np.dot(trans_mat,a_veh)
        #aa_out =np.dot(trans_mat,aa_veh)
        for ii in v_out:
            udp_list.append(ii[0,0])
        for ii in a_out:
            udp_list.append(ii[0,0])
        for ii in aa_veh:
            udp_list.append(ii[0,0])
        for ii in x_veh:
            udp_list.append(ii[0,0])
        for ii in x_a_veh:
            udp_list.append(ii[0,0])
        udp_list.append(waypoint_o.road_id)
        udp_list.append(waypoint_o.s)#not used yet
        udp_list.append(waypoint_o.lane_id)
        return udp_list,waypoint_o
