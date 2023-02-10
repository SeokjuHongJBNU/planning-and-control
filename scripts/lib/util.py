import os
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