from lidar import LIDAR
from lib.morai_udp_parser import udp_parser,udp_sender
import math
import time
import os,json

path = os.path.dirname( os.path.abspath( __file__ ) )  # current file's path

with open(os.path.join(path,("params.json")),'r') as fp :  # current path + file name
    params = json.load(fp) 

params_lidar = params["params_lidar"]
params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]

class E_STOP :

    def __init__(self,params_lidar):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.lidar=LIDAR(params_lidar)
        self.cnt = []
        self._is_status=False
        self.distance = []
        # 거리 임계값
        self.dist_thresh = 12
        
        while not self._is_status:
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True

    def main_loop(self):
        if self.lidar.udp_lidar.is_lidar == True:
            self.lidar.lidar_call_back()
            # lidar으로 받아오는 steer 값
            if self.lidar.is_object == True:
                cluster_position = self.lidar.cluster_coords# degree     
                for point in cluster_position:
                    self.distance.append(math.sqrt(point[0]**2 + point[1]**2))
            else :
                self.distance = []
                
            ctrl_mode = 2 # 2 = AutoMode / 1 = KeyBoard
            Gear = 4 # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
            cmd_type = 2 # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration        
            send_velocity = 25 #cmd_type이 2일때 원하는 속도를 넣어준다.
            acceleration = 0 #cmd_type이 3일때 원하는 가속도를 넣어준다.     
            accel=0
            brake=0
            self.lidar.display_info()
            
            # 긴급제동 거리 12m로 제한
            if self.distance and self.lidar.is_object and min(self.distance) < self.dist_thresh:
                self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,0,acceleration,accel,10,0])
            else : 
                self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,0])
            

if __name__ == "__main__":
    adas = E_STOP(params_lidar)
    while True :
        adas.main_loop()