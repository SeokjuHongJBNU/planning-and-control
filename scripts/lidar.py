import numpy as np
import time
from lib.lidar_util import UDP_LIDAR_Parser
from lib.morai_udp_parser import udp_parser
import os,json
import open3d as o3d

path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params_connect=params["params"]
user_ip = params_connect["user_ip"]
lidar_port = params_connect["lidar_dst_port"]
params_lidar = params["params_lidar"]

print(user_ip,lidar_port)

class LIDAR():
    def __init__(self,params_lidar):
        self.udp_lidar = UDP_LIDAR_Parser(user_ip, lidar_port, params_lidar=params_lidar)
        self.pcd_info = PCD()
        self.n_clusters = 0
        self.cluster_coords = None
        self.is_object = False
        
    def lidar_call_back(self):
        ######## lidar data
        x=self.udp_lidar.x
        y=self.udp_lidar.y
        z=self.udp_lidar.z

        #raw point cloud (57600, 3)
        points = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1).T.astype(np.float32)
        #point cloud (3,57600)
        #print("point clouds",points,points.shape)

        #numpy to point cloud data
        self.pcd_info.point_np2pcd(points)
        self.pcd_info.get_origin_point(points)
        #voxelize
        self.pcd_info.Voxelize()
        
        height,width = 20,2
        #ROI filtering
        self.pcd_info.ROI_filtering(height,width)
        
        # DBscan clustering
        if self.pcd_info.pcd.points :
            # points shape (,3)
            #self.pcd_info.Display_pcd(self.pcd_info.pcd)
            self.n_clusters, self.cluster_coords = self.pcd_info.o3d_DBscan()
            self.is_object = True
            # time.sleep(1)
        else : 
            self.n_clusters, self.cluster_coords = 0, None
            self.is_object = False
            # time.sleep(1)
            

    def display_info(self):
        print(f"number of point cloud data : {self.pcd_info.pcd_np.shape}")
        print(f"number of cluster : {self.n_clusters}\ncluster coordinate :\n{self.cluster_coords}")

class PCD:
    def __init__(self):
        self.origin_pcd = o3d.geometry.PointCloud()
        self.pcd =  o3d.geometry.PointCloud()
        self.pcd_np = None
        self.pcd_center = []
    
    def get_origin_point(self,points_np):
        self.pcd_np = points_np.T        
        self.origin_pcd.points = o3d.utility.Vector3dVector(self.pcd_np)

    def point_np2pcd(self, points_np):
        self.pcd_np = points_np.T        
        self.pcd.points = o3d.utility.Vector3dVector(self.pcd_np)
        
    def Voxelize(self):
        #print(f"Points before downsampling: {len(self.pcd.points)} ")
        # Points before downsampling: 115384 
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.2)
        #print(f"Points after downsampling: {len(self.pcd.points)}")
        self.pcd_np = np.asarray(self.pcd.points)
    
    def Display_pcd(self, pcd):
        o3d.visualization.draw_geometries([pcd])
        
    def Write_pcd(self, file_name):
        output_file = file_name
        with open(output_file, 'wt', newline='\r\n', encoding='UTF-8') as csvfile:
            for line in self.pcd_np:
                csvfile.write(str(line) + '\n')
                
    def channel_filtering(self, channel_select):
        channel_list = np.array([[-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]])
        channel_idx = np.where(channel_list == channel_select)
        channel_idx = channel_idx[1][0]
        
        self.pcd_np = self.pcd_np.T[channel_idx::16,:]
        self.point_np2pcd(self.pcd_np)
        
    def ROI_filtering(self,ROIheight,ROIwidth):
        #point shape (3,)
        points = self.pcd_np.T

        points = np.delete(points,np.where(points[2,:]<-0.5),axis=1)
        points = np.delete(points,np.where(points[2,:]>0.7),axis=1)
        
        points = np.delete(points,np.where(points[1,:]>ROIheight),axis=1)
        points = np.delete(points,np.where(points[1,:]<1),axis=1)
        
        points = np.delete(points,np.where(points[0,:]>ROIwidth),axis=1)
        points = np.delete(points,np.where(points[0,:]<-ROIwidth),axis=1)

        self.pcd_np = points.T
        self.point_np2pcd(points)
        
    
    def get_pcd_center(self,idx):
        # np2pcd
        pcd =  o3d.geometry.PointCloud()
        pcd_center = o3d.geometry.PointCloud()
        pcd_bbox_center = o3d.geometry.PointCloud()
        
        pcd_np = self.pcd_np[idx,:]   
        pcd_np = np.squeeze(pcd_np)  
        pcd.points = o3d.utility.Vector3dVector(pcd_np)  
        pcd_center_np = pcd.get_center() 
        pcd_center_np = pcd_center_np[np.newaxis]
        pcd_center.points = o3d.utility.Vector3dVector(pcd_center_np) 
        
        pcd_bbox = pcd.get_axis_aligned_bounding_box()  
        pcd_bbox_center_np = pcd_bbox.get_center()
        pcd_center_np = pcd.get_center() 
        
        #pcd_bbox_center_np = pcd_bbox_center_np[np.newaxis]
        # pcd_center_np = pcd_center_np[np.newaxis]
        # pcd_bbox_center.points = o3d.utility.Vector3dVector(pcd_bbox_center_np) 
        # pcd_center.points = o3d.utility.Vector3dVector(pcd_center_np) 
          
        pcd_bbox.color = (1, 0, 0)

        # o3d.visualization.draw_geometries([pcd, pcd_bbox_center, pcd_bbox,pcd_center])

        return pcd_bbox_center_np, pcd_bbox
        
    def o3d_DBscan(self):
        """DBscan을 통해 roi영역의 pointcloud를 clustering하여 객체 검출

        Returns:
            n_clusters_(int) : clustering 개수
            pcd_center_sorted(np.array) : clustering 의 중점 좌표 
        """
        # create model and prediction
        self.labels = self.pcd.cluster_dbscan(eps=1.0, min_points=10)
        # Number of clusters in labels, ignoring noise if present.
        print(self.labels)
        n_clusters_ = len(set(self.labels)) - (1 if -1 in self.labels else 0)
        n_noise_ = list(self.labels).count(-1)
        self.labels = np.asarray(self.labels)
        
        center_point = []
        pcd_center = []
        pcd_bbox = []
        for label in range(len(set(self.labels[self.labels!=-1]))):
            idx = np.where(self.labels==label)
            # mean point 
            #center_point.append(np.mean(self.pcd_np[idx,:],axis=1))
            # 3d bbox center point
            pc, pb = self.get_pcd_center(idx)
            pcd_center.append(pc)
            pcd_bbox.append(pb)
        
        center_points_np = np.array(center_point)
        pcd_center_np = np.array(pcd_center)
        center_points_np = np.squeeze(center_points_np)

        # (n,3)
        # print(pcd_center_np.shape)
        # print(center_points_np.shape)
        
        if pcd_center_np.shape[0] == 1:
            pcd_center_sorted = pcd_center_np
        else :
            try : pcd_center_sorted = pcd_center_np[pcd_center_np[:,1].argsort()]
            # Morai respwan bug exception
            except IndexError:
                pcd_center_sorted = pcd_center_np
                
        return n_clusters_, pcd_center_sorted

def main():
    lidar = LIDAR(params_lidar)
    obj=udp_parser(user_ip, params_connect["object_info_dst_port"],'erp_obj')
    ego=udp_parser(user_ip, params_connect["vehicle_status_dst_port"],'erp_status')
    
    while True :
        if lidar.udp_lidar.is_lidar ==True:
            # data parsing
            ###### obj info
            obj_data=obj.get_data()
            obj_data_np = np.array(obj_data)
            try : 
                obj_coords = obj_data_np[:,2:5]
            except IndexError :
                obj_coords = obj_data_np[2:5]
            #print("obj",obj_coords)    
            ####### ego info
            status_data = ego.get_data()
            status_data_np = np.array(status_data)
            ego_coords = status_data_np[12:15]
            
            # compare with sim object coordinate
            sim_coord = obj_coords - ego_coords
            print("obj",sim_coord)    
            
            #lidar call_back function
            lidar.lidar_call_back()
            # compare with simulation information
            lidar.display_info()
            #print(f"simulation object position :\n{sim_coord}\n")
            
if __name__ == '__main__':
    main()
