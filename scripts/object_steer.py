from lib.morai_udp_parser import udp_parser,udp_sender
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json


path = os.path.dirname( os.path.abspath( __file__ ) )

## define variables
change_rad_deg = 180/pi   # change radian scale to degree scale
first_head = 0            # memory variable for first(return) heading angle
ctn = 0                   # only for take first heading angle
return_rate = 0.4           # return steering angle 
avoid_rate = 0.4            # avoid steer angle
threshold_deg = 140      # when car avoids object it will maintane steer until degree become threshold deg
threshold_head = 5        # after the event, car's heading will return to first heading

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]



class go_straight :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.obj=udp_parser(user_ip, params["object_info_dst_port"],'erp_obj')
        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.safe_distance = 5  # 차량 안전거리

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
        global first_head
        self.timer=threading.Timer(0.001,self.main_loop)
        self.timer.start()
        
        status_data=self.status.get_data()
        obj_data=self.obj.get_data()
        obj_data= obj_data[0]
        
        
        obj_pos_x = obj_data[2]      # 장애물 x 좌표
        obj_pos_y = obj_data[3]      # 장애물 y 좌표
        obj_size_x = obj_data[6]     # 장애물 x 축 방향 길이(가로)
        obj_size_y = obj_data[7]     # 장애물 y 축 방향 길이(세로)
        position_x=status_data[12]   # 차량의 현재 x 좌표
        position_y=status_data[13]   # 차량의 현재 y 좌표
        heading=status_data[17]


        car2obj_dis = sqrt(pow((obj_pos_x - position_x),2) + pow((obj_pos_y - position_y),2))
        threshold = sqrt(pow(obj_size_x,2)+pow(obj_size_y,2)) + self.safe_distance

        if ctn == 0:
            first_head = heading

        x = position_x - obj_pos_x   # 장애물 to 차량 인 벡터 구하기
        y = position_y - obj_pos_y

        ## 추가 설명 참조##
        rad = atan2(y,x)
        deg = rad*change_rad_deg-heading     # 앞서구한 벡터와 heading 사이의 각도

        if deg > 180:         # 각도의 범위를 -180 to 180으로 scaling
            deg = deg - 360
        elif deg < -180:
            deg = 360 + deg

        deg_m = abs(deg)
        
        ctrl_mode = 2
        Gear = 4
        cmd_type = 1
        send_velocity = 0 #cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0 #cmd_type이 3일때 원하는 가속도를 넣어준다.
        accel=1
        brake=0
        steering_angle=0

        if car2obj_dis < threshold and deg_m > threshold_deg: #안전거리,안전각도 조건
            if deg > 0 :
                steering_angle= -1*avoid_rate
                
            else:
                steering_angle= avoid_rate
                
        
        if steering_angle == 0:  #회피 후 초기 heading으로 돌아가기 위한 조건식
            if heading > first_head + threshold_head or heading < first_head - threshold_head:
                    steering_angle = return_rate*abs(heading-60)/(heading-60)
        
        
        ctn = ctn +1

        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])


if __name__ == "__main__":


    kicty=go_straight()
    while True :
        pass