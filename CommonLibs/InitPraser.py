import configparser
import os

class InitSetting:
    def __init__(self,algorithm):
        self.config = configparser.ConfigParser()
        root = os.getcwd()
        self.config.read(root+"/ConfigFiles/"+algorithm+".ini",encoding='utf-8')
        self.map_name = self.config.get('Simulation','MapName')
        self.sync_step = float(self.config.get('Simulation','SyncStep'))
        self.spectator_hight=float(self.config.get('Simulation','SpectatorHeight'))
        self.left = int(self.config.get("LeftLane","CAR_Num"))
        self.leftDist = int(self.config.get("LeftLane","Diff_Dist"))
        self.right = int(self.config.get("RightLane","CAR_Num"))
        self.rightDist = int(self.config.get("RightLane","Diff_Dist"))
        self.front= int(self.config.get("MainLane","CAR_Num"))
        self.frontDist = int(self.config.get("MainLane","Diff_Dist"))
        self.rx_ip = self.config.get("RX_IP","Address")
        self.tx_ip = self.config.get("TX_IP","Address") 
        self.pos_x = float(self.config.get("Init_Pos","x"))
        self.pos_y = float(self.config.get("Init_Pos","y"))
        self.pos_z = float(self.config.get("Init_Pos","z")) 
        self.pos_pitch = float(self.config.get("Init_Pos","pitch")) 
        self.pos_roll = float(self.config.get("Init_Pos","roll")) 
        self.pos_yaw = float(self.config.get("Init_Pos","yaw")) 
        self.red_time = float(self.config.get("TrafficLight","RedTime"))
        self.green_time = float(self.config.get("TrafficLight","GreenTime"))
        self.yellow_time = float(self.config.get("TrafficLight","YellowTime"))
        self.v2x_switch = self.config.get("Simulation","V2X_Switch")
      