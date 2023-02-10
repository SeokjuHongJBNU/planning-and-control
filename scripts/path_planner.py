from lib.morai_udp_parser import udp_parser,udp_sender
from lib.util import pathReader,findLocalPath,purePursuit,Point
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json


path = os.path.dirname( os.path.abspath( __file__ ) )  
len_ob2car = float("inf")
ctn = 0
local_path = []
Avoid_Radius = 0

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp) 

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]



class ppfinal :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.obj=udp_parser(user_ip, params["object_info_dst_port"],'erp_obj')
        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.safety_distance = 5 # 안전거리
        self.dis4change_path = 6 # Local Path 추종 지점
        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read('new_curve.txt') 
        self.pure_pursuit=purePursuit() 
  

        self._is_status=False
        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True


        self.main_loop()


    
    def main_loop(self):
        global ctn
        global len_ob2car
        global local_path
        global Avoid_Radius

        self.timer=threading.Timer(0.001,self.main_loop)
        self.timer.start()
        
        status_data=self.status.get_data()
        obj_data=self.obj.get_data()
        position_x=status_data[12]
        position_y=status_data[13]
        position_z=status_data[14]
        heading=status_data[17]     
        velocity=status_data[18]

# 장애물이 인지되면 장애물 정보를 저장합니다.
# 만일 처음 장애물이 인지되었다면 Global Path를 탐색하며 미리 Local Path를 생성합니다.
# 장애물과 차의 거리가 2*안전거리 이내라면 Local Path를 추종합니다.
        if obj_data != []: 
            obj_data= obj_data[0]
            obj_pos_x = obj_data[2]
            obj_pos_y = obj_data[3]
            obj_size_x = obj_data[6]
            obj_size_y = obj_data[7]
            Avoid_Radius = sqrt(pow(obj_size_x,2)+pow(obj_size_y,2))
            len_ob2car = sqrt(pow((obj_pos_x - position_x),2) + pow((obj_pos_y - position_y),2)) 
            # if ctn == 0:
            #     ctn = ctn + 1
                # local_path =findLocalPath(self.global_path,Avoid_Radius,self.safety_distance,obj_pos_x,obj_pos_y)
            local_path =findLocalPath(self.global_path,Avoid_Radius,self.safety_distance,obj_pos_x,obj_pos_y)

        if len_ob2car <= Avoid_Radius+self.dis4change_path: 
            self.pure_pursuit.getPath(local_path)

        

# 그렇지 않다면 Global Path를 추종합니다.
        else:
            self.pure_pursuit.getPath(self.global_path)


        self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)
      
        ctrl_mode = 2 
        Gear = 4 
        cmd_type = 1     
        send_velocity = 0 
        acceleration = 0    
        accel=1
        brake=0

        steering_angle=self.pure_pursuit.steering_angle() 
        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])
            

if __name__ == "__main__":


    kicty=ppfinal()
    while True :
        pass