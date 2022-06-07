import carla

class TFL():
    def __init__(self,world):
        actors = world.get_actors()         
        self.traffic_lights={}
        for actor in actors:            
            if('traffic_light' in actor.type_id):
                self.traffic_lights[actor.get_opendrive_id()]=actor
    def traffic_light_time_set(self,init_setting):    
        for key,value in self.traffic_lights.items():
                    value.set_green_time(init_setting.green_time)
                    value.set_yellow_time(init_setting.yellow_time)
                    value.set_red_time(init_setting.red_time)
    def traffic_light_set(self,received_data):
        light_value=received_data[3]
        for key,traffic_light in self.traffic_lights.items():
                if received_data[2] == float(key):
                    if light_value ==1:
                        traffic_light.set_state(carla.TrafficLightState.Red)
                    elif light_value ==2:
                        traffic_light.set_state(carla.TrafficLightState.Yellow)
                    elif light_value ==3:
                        traffic_light.set_state(carla.TrafficLightState.Green)
                    else:
                        traffic_light.set_state(carla.TrafficLightState.Off)
    def get_traffic_light(self,received_data):
        '''
        received_data[2]: traffic light id
        received_data[4] current phase time
        '''
        udp_list=[]
        traffic_light=self.traffic_lights[str(int(received_data[2]))]
        if (traffic_light.state==carla.TrafficLightState.Red):    
            
            udp_list.append(0)   
            udp_list.append(received_data[4])
            udp_list.append(traffic_light.get_red_time()-received_data[4])
            
        elif (traffic_light.state ==carla.TrafficLightState.Yellow):
            
            udp_list.append(2)       
            udp_list.append(received_data[4])
            udp_list.append(traffic_light.get_yellow_time()-received_data[4])
            
        elif (traffic_light.state ==carla.TrafficLightState.Green):
            
            udp_list.append(1) 
            udp_list.append(received_data[4])
            udp_list.append(traffic_light.get_green_time()-received_data[4])  
            
        else:
            udp_list.append(3)
            udp_list.append(0)
            udp_list.append(0)
            received_data[4]=0
        return udp_list
