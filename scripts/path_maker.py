from lib.morai_udp_parser import udp_parser # 정리된 데이터를 가져다 사용하기위한 import
import time  
import threading
from math import cos,sin,sqrt,pow,atan2,pi
import os,json

path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
status_port = params["vehicle_status_dst_port"]
path_folder_name = params["make_path_folder_name"]
path_file_name = params["make_path_file_name"]

class path_maker :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.file_path=os.path.dirname( os.path.abspath( __file__ ) )
        self.file_path = os.path.normpath(os.path.join(self.file_path, '..'))

        
        self.full_path = self.file_path+'/'+path_folder_name+'/'+path_file_name
        self.prev_x = 0
        self.prev_y = 0
        
        self._is_status=False
        print(not self._is_status)
        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True

        self.main_loop()


    
    def main_loop(self):
        self.timer=threading.Timer(0.10,self.main_loop)
        self.timer.start()
        f=open(self.full_path, 'a')
        
        status_data=self.status.get_data()
        position_x=status_data[12]
        position_y=status_data[13]
        position_z=status_data[14]

        
        distance = sqrt(pow(position_x-self.prev_x,2)+pow(position_y-self.prev_y,2))
        if distance > 0.3 :
            data = '{0}\t{1}\t{2}\n'.format(position_x,position_y,position_z)
            f.write(data)
            self.prev_x = position_x
            self.prev_y = position_y
            print(position_x,position_y)
            f.close()



if __name__ == "__main__":

    path=path_maker()
    while True :
        pass