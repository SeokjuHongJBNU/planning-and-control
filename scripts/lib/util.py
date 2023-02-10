import os
import numpy as np
from math import cos,sin,sqrt,pow,atan2,acos,pi
class Point() :
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class pathReader :

    def __init__(self):
       self.file_path=os.path.dirname( os.path.abspath( __file__ ) )
       self.file_path = os.path.normpath(os.path.join(self.file_path, '../..'))
 


    def read(self,file_name):
        out_path=[]
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')

        line=openFile.readlines()

        for i in line :
            pose=[]
            postion=i.split()
            pose.append(float(postion[0]))
            pose.append(float(postion[1]))
            pose.append(float(postion[2]))
            out_path.append(pose)
            
        openFile.close()
        return out_path

class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=2
        self.lfd=5
        self.min_lfd=5
        self.max_lfd=30
        self.steering=0
        
    def getPath(self,path):
        self.path=path 
 
    
    # 차량의 정보를 불러오는 함수
    # 실체 차량 serial 값을 통해 얻은 속도, imu, gps 를 통해 얻은 yaw값, 차량의 위치
    def getEgoStatus(self,position_x,position_y,position_z,velocity,heading):

        self.current_vel=velocity  #kph
        self.vehicle_yaw=heading/180*pi   # rad
        self.current_postion.x=position_x
        self.current_postion.y=position_y
        self.current_postion.z=position_z

    def steering_angle(self):
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        if self.current_vel*0.3 < self.min_lfd:
            self.lfd = self.min_lfd
        elif self.current_vel*0.3 > self.max_lfd:
            self.lfd = self.max_lfd-1
        else:
            self.lfd = self.current_vel * 0.3

        for i in range(len(self.path)) :
            pathpoint = self.path[i]
            rel_x= pathpoint[0] - vehicle_position.x
            rel_y= pathpoint[1] - vehicle_position.y
            s = sqrt(rel_x*rel_x + rel_y*rel_y)
            if s > self.min_lfd and s < self.max_lfd and s > self.lfd:
                dot_x = rel_x*cos(self.vehicle_yaw) + rel_y*sin(self.vehicle_yaw)
                dot_y = rel_x*sin(self.vehicle_yaw) - rel_y*cos(self.vehicle_yaw)
                if dot_x > 0 :             
                    alpha=atan2(dot_y,dot_x)
                    self.forward_point=pathpoint
                    self.is_look_forward_point=True
                    break              

        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(alpha)),s)
            return self.steering #deg
        else : 
            print("There is no waypoint at front")
            return 0

class Stanley :
    def __init__(self):
        self.control_gain = 2
        self.forward_point=Point()
        self.current_postion=Point()
        self.vehicle_length=2
        self.min_steer=-30*pi/180
        self.max_steer=30*pi/180
        self.steering=0
        
    def getPath(self,path):
        self.path=path 
 
    
    
    def getEgoStatus(self,position_x,position_y,position_z,velocity,heading):

        self.current_vel=velocity  #mps
        self.vehicle_yaw=heading/180*pi # rad
        self.current_postion.x=position_x
        self.current_postion.y=position_y
        self.current_postion.z=position_z




    def steering_angle(self):
        vehicle_position=self.current_postion
        self.is_look_forward_point= False
        min_dist = float("inf")
        min_idx = 0

        front_x = vehicle_position.x + self.vehicle_length*cos(self.vehicle_yaw)
        front_y = vehicle_position.y + self.vehicle_length*sin(self.vehicle_yaw)

        for i in range(len(self.path)):
            wp = self.path[i]
            dx = front_x - wp[0]
            dy = front_y - wp[1]
            dist = sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        
        if min_idx < len(self.path)-2:
            tangency = atan2(self.path[min_idx+1][1]-self.path[min_idx][1],self.path[min_idx+1][0]-self.path[min_idx][0])
        else:
            tangency = atan2(self.path[min_idx][1]-self.path[min_idx-1][1],self.path[min_idx][0]-self.path[min_idx-1][0])

        psi = tangency - self.vehicle_yaw
        vectorr = [cos(tangency + pi/2), sin(tangency + pi/2)]
        dx = self.path[min_idx][0] - front_x
        dy = self.path[min_idx][1] - front_y
        cte = np.dot([dx,dy],vectorr)
        cte_term = atan2(self.control_gain*cte,self.current_vel)
        self.steering = psi + cte_term


        while self.steering > pi:
            self.steering -= 2.0*pi
        
        while self.steering < -pi:
            self.steering += 2.0*pi

        self.steering = -1*np.clip(self.steering,self.min_steer,self.max_steer)

        return self.steering


def findLocalPath(ref_path,Avoid_Radius,safety_distance,obj_pos_x,obj_pos_y):
    out_path=[]
    threshold = []
    t_idx_min = 0
    t_idx_max = 0

    for i in range(len(ref_path)): #Global path를 탐색하며 Threshold 안에 속한 waypoint의 index를 찾습니다.
        dx = ref_path[i][0] - obj_pos_x # 시점:장애물, 종점:Waypoint인 벡터 생성 
        dy = ref_path[i][1] - obj_pos_y
        dis = sqrt(dx*dx + dy*dy)    # 벡터 크기 구하기
        if dis < Avoid_Radius+safety_distance:    # Threshold 원의 반지름보다 작다면
            threshold.append(i)     # 인덱스 저장
    t_idx_min = min(threshold)      # 저장된 인덱스 중 최대 최소 추출
    t_idx_max = max(threshold)      # 최소~최대 인덱스 사이의 점들은 밀어내야 하기 때문입니다.

    for i in range(t_idx_min,t_idx_max+200): # 앞서 구한 인덱스를 기반으로 Local Path를 구합니다.
        pose = []
        dist = sqrt(pow((ref_path[i][0]-obj_pos_x),2)+pow((ref_path[i][1]-obj_pos_y),2)) 
        if i < t_idx_max and dist <= Avoid_Radius+safety_distance:
            newpoint_x = ref_path[i][0] - obj_pos_x  # 장애물 벡터를 구합니다.
            newpoint_y = ref_path[i][1] - obj_pos_y 
            slid_ang = atan2(newpoint_y,newpoint_x)      # 벡터의 위상을 구합니다.
            newpoint_x = (Avoid_Radius+safety_distance)*cos(slid_ang) + obj_pos_x# waypoint를 밀어줍니다.
            newpoint_y = (Avoid_Radius+safety_distance)*sin(slid_ang) + obj_pos_y
            
            pose.append(newpoint_x) # Local Path에 추가해줍니다.
            pose.append(newpoint_y)
            out_path.append(pose)      
        else:
            pose.append(ref_path[i][0])
            pose.append(ref_path[i][1])              
            out_path.append(pose)

    return out_path   # Local Path를 산출합니다.