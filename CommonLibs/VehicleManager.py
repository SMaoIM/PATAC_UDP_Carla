import carla
from carla import Transform,Location,Rotation
import random
import math
from CommonLibs.UniversalLib import lane_color,lane_mark
from Localization.LocalizationManager import vehicle_loc



class EgoVehicle:
    '''
    Parameter
    ----------------------------------
    world: carla.World object
    init_setting: InitSetting object, including all setup for simulation
    received_data: received date from Simulink by UDP
    map: carla.Map object
    
    '''
    def __init__(self,world,init_setting):
        blueprint = random.choice(world.get_blueprint_library().filter('charger_2020'))    
        blueprint.set_attribute('role_name','ego_vehicle')      
        blueprint.set_attribute('color','0,0,0')    
        if init_setting.map_name == 'yangguo03' or init_setting.map_name=='4way':
            init_setting.pos_y+=23
        spawn_point = Transform(Location(x=init_setting.pos_x ,y=init_setting.pos_y, z=init_setting.pos_z),
                      Rotation(pitch=init_setting.pos_pitch, yaw=init_setting.pos_yaw, roll=init_setting.pos_roll))
        self.ego_vehicle = world.spawn_actor(blueprint, spawn_point) 
        self.spectator=world.get_spectator()
        self.spectator_hight=init_setting.spectator_hight
    def spectator_set(self):
        transform = self.ego_vehicle.get_transform()
        self.spectator.set_transform(carla.Transform(transform.location + carla.Location(z=self.spectator_hight),
                                                     carla.Rotation(pitch=-90)))
        return transform
    def ego_motion_set(self,received_data):
        transform=self.ego_vehicle.get_transform()
        veh_spd=carla.Vector3D(x=received_data[0]*math.cos(transform.rotation.pitch/57.3)*math.cos(transform.rotation.yaw/57.3),
                                   y=received_data[0]*math.cos(transform.rotation.pitch/57.3)*math.sin(transform.rotation.yaw/57.3),
                                   z=received_data[0]*math.sin(transform.rotation.pitch/57.3))
        self.ego_vehicle.set_target_velocity(veh_spd)
        self.ego_vehicle.apply_control(carla.VehicleControl(steer =-received_data[1]))

    def get_roadinfo(self,map):
        udp_list=[]
        #get all the info of ego vehicles, including motion and position and lane info
        udp_list,waypoint_o=vehicle_loc(map,self.ego_vehicle)
        udp_list.append(lane_mark(waypoint_o.left_lane_marking.type))
        udp_list.append(lane_color(waypoint_o.left_lane_marking.color))
        udp_list.append(lane_mark(waypoint_o.right_lane_marking.type))
        udp_list.append(lane_color(waypoint_o.right_lane_marking.color))
        return udp_list

class TargetVehicle:
    '''
    Parameter
    ----------------------------------
    world: carla.World object
    init_setting: InitSetting object, including all setup for simulation
    received_data: received date from Simulink by UDP
    
    '''
    def __init__(self,world,init_setting) :
        self.vehicles=[]
        #front lane
        x0=init_setting.pos_x+init_setting.frontDist*math.cos(init_setting.pos_yaw/57.3)+5*math.cos(init_setting.pos_yaw/57.3)
        y0=init_setting.pos_y+init_setting.frontDist*math.sin(init_setting.pos_yaw/57.3)+5*math.sin(init_setting.pos_yaw/57.3)

        if(init_setting.front):
            for vv in range(init_setting.front):
                blueprint1 = random.choice(world.get_blueprint_library().filter('charger_2020')) 
                blueprint1.set_attribute('color','255,0,0')          
                spawn_point1 = carla.Transform(carla.Location(x=x0 ,y=y0, z=init_setting.pos_z),carla.Rotation(pitch=init_setting.pos_pitch, yaw=init_setting.pos_yaw, roll=init_setting.pos_roll))
                player1 = world.spawn_actor(blueprint1, spawn_point1)                      
                x0+=50*math.cos(init_setting.pos_yaw/57.3)
                y0+=50*math.sin(init_setting.pos_yaw/57.3)
                self.vehicles.append(player1)  
                if init_setting.v2x_switch=='off':
                    player1.set_autopilot()

        #left lane
        x0=init_setting.pos_x+init_setting.leftDist*math.cos(init_setting.pos_yaw/57.3)+5*math.cos(init_setting.pos_yaw/57.3)
        y0=init_setting.pos_y+init_setting.leftDist*math.sin(init_setting.pos_yaw/57.3)+5*math.sin(init_setting.pos_yaw/57.3)
        if(init_setting.left):
            for vv in range(init_setting.left):
                blueprint1 = random.choice(world.get_blueprint_library().filter('charger_2020'))
                blueprint1.set_attribute('color','0,255,0')           
                spawn_point1 = carla.Transform(carla.Location(x=x0+3.5*math.sin(init_setting.pos_yaw/57.3) ,y=y0-3.5*math.cos(init_setting.pos_yaw/57.3), z=init_setting.pos_z),carla.Rotation(pitch=init_setting.pos_pitch, yaw=init_setting.pos_yaw, roll=init_setting.pos_roll))
                player1 = world.spawn_actor(blueprint1, spawn_point1)       
                x0+=50*math.cos(init_setting.pos_yaw/57.3)
                y0+=50*math.sin(init_setting.pos_yaw/57.3)
                self.vehicles.append(player1)
                if init_setting.v2x_switch=='off':
                    player1.set_autopilot()
        
        #right lane
        x0=init_setting.pos_x+init_setting.rightDist*math.cos(init_setting.pos_yaw/57.3)+5*math.cos(init_setting.pos_yaw/57.3)
        y0=init_setting.pos_y+init_setting.rightDist*math.sin(init_setting.pos_yaw/57.3)+5*math.sin(init_setting.pos_yaw/57.3)
        if(init_setting.right):
            for vv in range(init_setting.right):
                blueprint1 = random.choice(world.get_blueprint_library().filter('charger_2020'))      
                blueprint1.set_attribute('color','0,0,255')     
                spawn_point1 = carla.Transform(carla.Location(x=x0-3.5*math.sin(init_setting.pos_yaw/57.3) ,y=y0+3.5*math.cos(init_setting.pos_yaw/57.3), z=init_setting.pos_z),carla.Rotation(pitch=init_setting.pos_pitch, yaw=init_setting.pos_yaw, roll=init_setting.pos_roll))
                player1 = world.spawn_actor(blueprint1, spawn_point1)       
                x0+=50*math.cos(init_setting.pos_yaw/57.3)
                y0+=50*math.sin(init_setting.pos_yaw/57.3)
                self.vehicles.append(player1)
                if init_setting.v2x_switch=='off':
                    player1.set_autopilot()
    def target_motion_set(self,received_data):
         for ii in range(len(self.vehicles)):
                    transform_tar=self.vehicles[ii].get_transform()
                    if ii<6:
                        tar_veh_spd=carla.Vector3D(x=received_data[ii+5]*math.cos(transform_tar.rotation.pitch/57.3)*math.cos(transform_tar.rotation.yaw/57.3),
                                    y=received_data[ii+5]*math.cos(transform_tar.rotation.pitch/57.3)*math.sin(transform_tar.rotation.yaw/57.3),
                                    z=received_data[ii+5]*math.sin(transform_tar.rotation.pitch/57.3))
                    else:
                        tar_veh_spd=carla.Vector3D(x=received_data[11]*math.cos(transform_tar.rotation.pitch/57.3)*math.cos(transform_tar.rotation.yaw/57.3),
                                    y=received_data[11]*math.cos(transform_tar.rotation.pitch/57.3)*math.sin(transform_tar.rotation.yaw/57.3),
                                    z=received_data[11]*math.sin(transform_tar.rotation.pitch/57.3))
                    self.vehicles[ii].set_target_velocity(tar_veh_spd)


class RandomVehicle:
    '''
    vehicle generation with random spawn points
    '''
    def __init__(self,world,map):
        blueprint = random.choice(world.get_blueprint_library().filter('charger_2020'))          
        blueprint.set_attribute('color','128,128,128')    
        spawn_point = random.choice(map.get_spawn_points())
        self.object_vehicle = world.spawn_actor(blueprint, spawn_point) 
        self.object_vehicle.set_autopilot()

