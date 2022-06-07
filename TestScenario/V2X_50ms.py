from V2X.TFL_module import TFL
from CommonLibs.UdpSetup import UdpCtrl
from CommonLibs.InitPraser import InitSetting
from CommonLibs.VehicleManager import EgoVehicle,TargetVehicle
from Perception.SensorManager import SemanticLidarSensor
import carla
import time

def RUN_TEST():
    vehicles=[]
    sensor_list=[]
    
    try:
        init_setting =InitSetting(algorithm='V2X_50ms')
        #set client
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        client.load_world(init_setting.map_name)
        world = client.get_world()
        map = world.get_map()

        # set synchorinized mode
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = init_setting.sync_step
        settings.synchronous_mode = True
        world.apply_settings(settings)
        #traffic manager
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        #udp setting
        udp_ctrl = UdpCtrl(init_setting)

        #create ego vehicle with sematic lidar
        ego_vehicle_object= EgoVehicle(world,init_setting)
        ego_vehicle=ego_vehicle_object.ego_vehicle
        vehicles.append(ego_vehicle)
        semantic_lidar_object = SemanticLidarSensor(ego_vehicle,world)
        sensor_list.append(semantic_lidar_object)

        #create a target vehicle
        target_vehicles_object = TargetVehicle(world,init_setting)
        target_vehicles = target_vehicles_object.vehicles
        for target_vehicle in target_vehicles:
            vehicles.append(target_vehicle)

        #get all traffic light info and set traffic light switch time 
        traffic_light_object = TFL(world)
        traffic_light_object.traffic_light_time_set(init_setting)
        world.tick()
        #tick the world to see the sceen created
        print(world.get_snapshot().timestamp.elapsed_seconds)
        print("***********************start simulation****************")
        ego_vehicle_object.spectator_set()
        world.tick()

        while(1):
            timein=time.time()
            #tick the world with specific spectator 
            world.tick()
            ego_vehicle_object.spectator_set()
            #udp communication with Simulink
            received_data=udp_ctrl.msg_receiver()  
            udp_ctrl.msg_sender(map,ego_vehicle_object,vehicles,semantic_lidar_object,traffic_light_object,init_setting)
            udp_ctrl.msg_sender_v2x(ego_vehicle,map)
            #set traffic light
            traffic_light_object.traffic_light_set(received_data)
            #set target vehichle speed
            target_vehicles_object.target_motion_set(received_data)
            #set ego vehicle speed&steering
            ego_vehicle_object.ego_motion_set(received_data)
            
            time_out=time.time()
            #print(time_out-timein)
    finally:
        world.apply_settings(original_settings)
        for vehiclex in vehicles:
            vehiclex.destroy()
        for sensor in sensor_list:
            sensor.destroy()
        #ob_sensor.destroy()
        print(world.get_snapshot().timestamp.elapsed_seconds)
        #print(time.time())
        udp_ctrl.destroy()
        print('********************finished*********************')

