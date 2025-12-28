from .mapping_utils.geometry import *
from .mapping_utils.preprocess import *
from .mapping_utils.projection import *
from .mapping_utils.path_planning import *
from matplotlib import colormaps
from .constants import *
import open3d as o3d
from PIL import Image

# Try to import GLEE_Percevior, but make it optional
try:
    from .cv_utils.image_percevior import GLEE_Percevior
    GLEE_AVAILABLE = True
except ImportError:
    GLEE_AVAILABLE = False
    print("[Mapper] Warning: GLEE_Percevior not available. Object detection will be disabled.")

D3_40_COLORS_RGB = np.array([
    [31,119,180],[255,127,14],[44,160,44],[214,39,40],[148,103,189],
    [140,86,75],[227,119,194],[127,127,127],[188,189,34],[23,190,207],
    [174,199,232],[255,187,120],[152,223,138],[255,152,150],[197,176,213],
    [196,156,148],[247,182,210],[199,199,199],[219,219,141],[158,218,229],
    [66,133,244],[219,68,55],[244,180,0],[15,157,88],[171,71,188],
    [0,172,193],[255,112,67],[153,204,0],[255,153,51],[255,51,153],
    [102,102,255],[102,255,102],[255,102,102],[255,255,102],[102,255,255],
    [255,102,255],[204,153,255],[204,255,153],[255,204,153],[153,204,255]
], dtype=np.uint8)

class Mapper:
    def __init__(self,
                 camera_intrinsic,
                 pcd_resolution=0.05,
                 grid_resolution=0.1,
                 grid_size=5,
                 floor_height=-0.8,
                 ceiling_height=0.8,
                 translation_func=habitat_translation,
                 rotation_func=habitat_rotation,
                 rotate_axis=[0,1,0],
                 device='cuda:0',
                 enable_object_detection=True):
        self.device = device
        self.camera_intrinsic = camera_intrinsic
        self.pcd_resolution = pcd_resolution
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.floor_height = floor_height
        self.ceiling_height = ceiling_height
        self.translation_func = translation_func
        self.rotation_func = rotation_func
        self.rotate_axis = np.array(rotate_axis)
        
        # Initialize object detector if available and enabled
        self.enable_object_detection = enable_object_detection and GLEE_AVAILABLE
        if self.enable_object_detection:
            self.object_percevior = GLEE_Percevior(device=device)
        else:
            self.object_percevior = None
            if enable_object_detection and not GLEE_AVAILABLE:
                print("[Mapper] Object detection requested but GLEE not available.")
        
        self.pcd_device = o3d.core.Device(device.upper())
    
    def reset(self,position,rotation):
        self.update_iterations = 0
        self.initial_position = self.translation_func(position)
        self.current_position = self.translation_func(position) - self.initial_position
        self.current_rotation = self.rotation_func(rotation)
        self.scene_pcd = o3d.t.geometry.PointCloud(self.pcd_device)
        self.navigable_pcd = o3d.t.geometry.PointCloud(self.pcd_device)
        self.object_pcd = o3d.t.geometry.PointCloud(self.pcd_device)
        self.object_entities = []
        self.trajectory_position = []
    
    def update(self,rgb,depth,position,rotation):
        self.current_position = self.translation_func(position) - self.initial_position
        self.current_rotation = self.rotation_func(rotation)
        self.current_depth = preprocess_depth(depth)
        self.current_rgb = preprocess_image(rgb)
        self.trajectory_position.append(self.current_position)
        # to avoid there is no valid depth value (especially in real-world)
        if np.sum(self.current_depth) > 0:
            camera_points,camera_colors = get_pointcloud_from_depth(self.current_rgb,self.current_depth,self.camera_intrinsic)
            world_points = translate_to_world(camera_points,self.current_position,self.current_rotation)
            self.current_pcd = gpu_pointcloud_from_array(world_points,camera_colors,self.pcd_device).voxel_down_sample(self.pcd_resolution)
        else:
            return
        
        # semantic masking and project object mask to pointcloud (only if object detector is available)
        current_object_entities = []
        if self.enable_object_detection and self.object_percevior is not None:
            classes,masks,confidences,visualization = self.object_percevior.perceive(self.current_rgb)
            self.segmentation = visualization[0]
            current_object_entities = self.get_object_entities(self.current_depth,classes,masks,confidences)
            self.object_entities = self.associate_object_entities(self.object_entities,current_object_entities)
            self.object_pcd = self.update_object_pcd()
        else:
            self.segmentation = self.current_rgb  # Use raw RGB as segmentation placeholder
            
        # pointcloud update
        self.scene_pcd = gpu_merge_pointcloud(self.current_pcd,self.scene_pcd).voxel_down_sample(self.pcd_resolution)
        self.scene_pcd = self.scene_pcd.select_by_index((self.scene_pcd.point.positions[:,2]>self.floor_height-0.2).nonzero()[0])
        self.useful_pcd = self.scene_pcd.select_by_index((self.scene_pcd.point.positions[:,2]<self.ceiling_height).nonzero()[0])
        
        # all the stairs will be regarded as navigable
        for entity in current_object_entities:
            if entity['class'] == 'stairs':
                self.navigable_pcd = gpu_merge_pointcloud(self.navigable_pcd,entity['pcd'])
        # geometry 
        current_navigable_point = self.current_pcd.select_by_index((self.current_pcd.point.positions[:,2]<self.floor_height).nonzero()[0])
        current_navigable_position = current_navigable_point.point.positions.cpu().numpy()
        standing_position = np.array([self.current_position[0],self.current_position[1],current_navigable_position[:,2].mean()])
        interpolate_points = np.linspace(np.ones_like(current_navigable_position)*standing_position,current_navigable_position,25).reshape(-1,3)
        interpolate_points = interpolate_points[(interpolate_points[:,2] > self.floor_height-0.2) & (interpolate_points[:,2] < self.floor_height+0.2)]
        interpolate_colors = np.ones_like(interpolate_points) * 100
        try:
            current_navigable_pcd = gpu_pointcloud_from_array(interpolate_points,interpolate_colors,self.pcd_device).voxel_down_sample(self.grid_resolution)
            self.navigable_pcd = gpu_merge_pointcloud(self.navigable_pcd,current_navigable_pcd).voxel_down_sample(self.pcd_resolution)
        except:
            self.navigable_pcd = self.useful_pcd.select_by_index((self.useful_pcd.point.positions[:,2]<self.floor_height).nonzero()[0])
       
        
        # try:
        #     self.navigable_pcd = self.navigable_pcd.voxel_down_sample(self.pcd_resolution)
        # except:
        #     self.navigable_pcd = self.useful_pcd.select_by_index((self.useful_pcd.point.positions[:,2]<self.floor_height).nonzero()[0])
        #print("Warning: hello world")
        # self.navigable_pcd = self.useful_pcd.select_by_index((self.useful_pcd.point.positions[:,2]<self.floor_height).nonzero()[0])
            
        # filter the obstacle pointcloud
        self.obstacle_pcd = self.useful_pcd.select_by_index((self.useful_pcd.point.positions[:,2]>self.floor_height+0.1).nonzero()[0])
        self.trajectory_pcd = gpu_pointcloud_from_array(np.array(self.trajectory_position),np.zeros((len(self.trajectory_position),3)),self.pcd_device)
        self.frontier_pcd = project_frontier(self.obstacle_pcd,self.navigable_pcd,self.floor_height+0.2,self.grid_resolution)
        self.frontier_pcd[:,2] = self.navigable_pcd.point.positions.cpu().numpy()[:,2].mean()
        self.frontier_pcd = gpu_pointcloud_from_array(self.frontier_pcd,np.ones((self.frontier_pcd.shape[0],3))*np.array([[255,0,0]]),self.pcd_device)
        self.update_iterations += 1
    
    def update_object_pcd(self):
        object_pcd = o3d.geometry.PointCloud()
        for entity in self.object_entities:
            points = entity['pcd'].point.positions.cpu().numpy()
            colors = entity['pcd'].point.colors.cpu().numpy()
            new_pcd = o3d.geometry.PointCloud()
            new_pcd.points = o3d.utility.Vector3dVector(points)
            new_pcd.colors = o3d.utility.Vector3dVector(colors)
            object_pcd = object_pcd + new_pcd
        try:
            return gpu_pointcloud(object_pcd,self.pcd_device)
        except:
            return self.scene_pcd
    
    def get_view_pointcloud(self,rgb,depth,translation,rotation):
        current_position = self.translation_func(translation) - self.initial_position
        current_rotation = self.rotation_func(rotation)
        current_depth = preprocess_depth(depth)
        current_rgb = preprocess_image(rgb)
        camera_points,camera_colors = get_pointcloud_from_depth(current_rgb,current_depth,self.camera_intrinsic)
        world_points = translate_to_world(camera_points,current_position,current_rotation)
        current_pcd = gpu_pointcloud_from_array(world_points,camera_colors,self.pcd_device).voxel_down_sample(self.pcd_resolution)
        return current_pcd
    
    def get_object_entities(self,depth,classes,masks,confidences):
        entities = []
        exist_objects = np.unique([ent['class'] for ent in self.object_entities]).tolist()
        for cls,mask,score in zip(classes,masks,confidences):
            if depth[mask>0].min() < 1.0 and score < 0.5:
                continue
            if cls not in exist_objects:
                exist_objects.append(cls)
            camera_points = get_pointcloud_from_depth_mask(depth,mask,self.camera_intrinsic)
            world_points = translate_to_world(camera_points,self.current_position,self.current_rotation)
            point_colors = np.array([D3_40_COLORS_RGB[exist_objects.index(cls)%40]]*world_points.shape[0])
            if world_points.shape[0] < 10:
                continue
            object_pcd = gpu_pointcloud_from_array(world_points,point_colors,self.pcd_device).voxel_down_sample(self.pcd_resolution)
            object_pcd = gpu_cluster_filter(object_pcd)
            if object_pcd.point.positions.shape[0] < 10:
                continue
            entity = {'class':cls,'pcd':object_pcd,'confidence':score}
            entities.append(entity)
        return entities
    
    def associate_object_entities(self,ref_entities,eval_entities):        
        for entity in eval_entities:
            if len(ref_entities) == 0:
                ref_entities.append(entity)
                continue
            overlap_score = []
            eval_pcd = entity['pcd']
            for ref_entity in ref_entities:
                if eval_pcd.point.positions.shape[0] == 0:
                    break
                cdist = pointcloud_distance(eval_pcd,ref_entity['pcd'])
                overlap_condition = (cdist < 0.1)
                nonoverlap_condition = overlap_condition.logical_not()
                eval_pcd = eval_pcd.select_by_index(o3d.core.Tensor(nonoverlap_condition.cpu().numpy(),device=self.pcd_device).nonzero()[0])
                overlap_score.append((overlap_condition.sum()/(overlap_condition.shape[0]+1e-6)).cpu().numpy())
            max_overlap_score = np.max(overlap_score)
            arg_overlap_index = np.argmax(overlap_score)
            if max_overlap_score < 0.25:
                entity['pcd'] = eval_pcd
                ref_entities.append(entity)
            else:
                argmax_entity = ref_entities[arg_overlap_index]
                argmax_entity['pcd'] = gpu_merge_pointcloud(argmax_entity['pcd'],eval_pcd)
                if argmax_entity['pcd'].point.positions.shape[0] < entity['pcd'].point.positions.shape[0] or entity['class'] in INTEREST_OBJECTS:
                    argmax_entity['class'] = entity['class']
                ref_entities[arg_overlap_index] = argmax_entity
        return ref_entities
    
    def get_obstacle_affordance(self):
        try:
            distance = pointcloud_distance(self.navigable_pcd,self.obstacle_pcd)
            affordance = (distance - distance.min())/(distance.max() - distance.min() + 1e-6)
            affordance[distance < 0.25] = 0
            return affordance.cpu().numpy()
        except:
            return np.zeros((self.navigable_pcd.point.positions.shape[0],),dtype=np.float32)
    
    def get_trajectory_affordance(self):
        try:
            distance = pointcloud_distance(self.navigable_pcd,self.trajectory_pcd)
            affordance = (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
            return affordance.cpu().numpy()
        except:
            return np.zeros((self.navigable_pcd.point.positions.shape[0],),dtype=np.float32)
    
    def get_semantic_affordance(self,target_class,threshold=0.1):
        semantic_pointcloud = o3d.t.geometry.PointCloud()
        for entity in self.object_entities:
            if entity['class'] in target_class:
                semantic_pointcloud = gpu_merge_pointcloud(semantic_pointcloud,entity['pcd'])
        try:
            distance = pointcloud_2d_distance(self.navigable_pcd,semantic_pointcloud) 
            affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
            affordance[distance > threshold] = 0
            affordance = affordance.cpu().numpy()
            return affordance
        except:
            return np.zeros((self.navigable_pcd.point.positions.shape[0],),dtype=np.float32)
    
    def get_gpt4v_affordance(self,gpt4v_pcd):
        try:
            distance = pointcloud_distance(self.navigable_pcd,gpt4v_pcd)
            affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
            affordance[distance > 0.1] = 0
            return affordance.cpu().numpy()
        except:
            return np.zeros((self.navigable_pcd.point.positions.shape[0],),dtype=np.float32)
    
    def get_action_affordance(self,action):
        try:
            if action == 'Explore':
                distance = pointcloud_2d_distance(self.navigable_pcd,self.frontier_pcd)
                affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
                affordance[distance > 0.2] = 0
                return affordance.cpu().numpy()
            elif action == 'Move_Forward':
                pixel_x,pixel_z,depth_values = project_to_camera(self.navigable_pcd,self.camera_intrinsic,self.current_position,self.current_rotation)
                filter_condition = (pixel_x >= 0) & (pixel_x < self.camera_intrinsic[0][2]*2) & (pixel_z >= 0) & (pixel_z < self.camera_intrinsic[1][2]*2) & (depth_values > 1.5) & (depth_values < 2.5)
                filter_pcd = self.navigable_pcd.select_by_index(o3d.core.Tensor(np.where(filter_condition==1)[0],device=self.navigable_pcd.device))
                distance = pointcloud_distance(self.navigable_pcd,filter_pcd)
                affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
                affordance[distance > 0.1] = 0
                return affordance.cpu().numpy()
            elif action == 'Turn_Around':
                R = np.array([np.pi,np.pi,np.pi]) * self.rotate_axis
                turn_extrinsic = np.matmul(self.current_rotation,quaternion.as_rotation_matrix(quaternion.from_euler_angles(R)))
                pixel_x,pixel_z,depth_values = project_to_camera(self.navigable_pcd,self.camera_intrinsic,self.current_position,turn_extrinsic)
                filter_condition = (pixel_x >= 0) & (pixel_x < self.camera_intrinsic[0][2]*2) & (pixel_z >= 0) & (pixel_z < self.camera_intrinsic[1][2]*2) & (depth_values > 1.5) & (depth_values < 2.5)
                filter_pcd = self.navigable_pcd.select_by_index(o3d.core.Tensor(np.where(filter_condition==1)[0],device=self.navigable_pcd.device))
                distance = pointcloud_distance(self.navigable_pcd,filter_pcd)
                affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
                affordance[distance > 0.1] = 0
                return affordance.cpu().numpy()
            elif action == 'Turn_Left':
                R = np.array([np.pi/2,np.pi/2,np.pi/2]) * self.rotate_axis
                turn_extrinsic = np.matmul(self.current_rotation,quaternion.as_rotation_matrix(quaternion.from_euler_angles(R)))
                pixel_x,pixel_z,depth_values = project_to_camera(self.navigable_pcd,self.camera_intrinsic,self.current_position,turn_extrinsic)
                filter_condition = (pixel_x >= 0) & (pixel_x < self.camera_intrinsic[0][2]*2) & (pixel_z >= 0) & (pixel_z < self.camera_intrinsic[1][2]*2) & (depth_values > 1.5) & (depth_values < 2.5)
                filter_pcd = self.navigable_pcd.select_by_index(o3d.core.Tensor(np.where(filter_condition==1)[0],device=self.navigable_pcd.device))
                distance = pointcloud_distance(self.navigable_pcd,filter_pcd)
                affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
                affordance[distance > 0.1] = 0
                return affordance.cpu().numpy()
            elif action == 'Turn_Right':
                R = np.array([-np.pi/2,-np.pi/2,-np.pi/2]) * self.rotate_axis
                turn_extrinsic = np.matmul(self.current_rotation,quaternion.as_rotation_matrix(quaternion.from_euler_angles(R)))
                pixel_x,pixel_z,depth_values = project_to_camera(self.navigable_pcd,self.camera_intrinsic,self.current_position,turn_extrinsic)
                filter_condition = (pixel_x >= 0) & (pixel_x < self.camera_intrinsic[0][2]*2) & (pixel_z >= 0) & (pixel_z < self.camera_intrinsic[1][2]*2) & (depth_values > 1.5) & (depth_values < 2.5)
                filter_pcd = self.navigable_pcd.select_by_index(o3d.core.Tensor(np.where(filter_condition==1)[0],device=self.navigable_pcd.device))
                distance = pointcloud_distance(self.navigable_pcd,filter_pcd)
                affordance = 1 - (distance - distance.min()) / (distance.max() - distance.min() + 1e-6)
                affordance[distance > 0.1] = 0
                return affordance.cpu().numpy()
            elif action == 'Enter':
                return self.get_semantic_affordance(['doorway','door','entrance','exit'])
            elif action == 'Exit':
                return self.get_semantic_affordance(['doorway','door','entrance','exit'])
            else:
                return np.zeros((self.navigable_pcd.point.positions.shape[0],),dtype=np.float32) 
        except:
            return np.zeros((self.navigable_pcd.point.positions.shape[0],),dtype=np.float32) 

    def get_objnav_affordance_map(self,action,target_class,gpt4v_pcd,complete_flag=False,failure_mode=False):
        if failure_mode:
            obstacle_affordance = self.get_obstacle_affordance()
            affordance = self.get_action_affordance('Explore')
            affordance = np.clip(affordance,0.1,1.0)
            affordance[obstacle_affordance == 0] = 0
            return affordance,self.visualize_affordance(affordance)
        elif complete_flag:
            affordance = self.get_semantic_affordance([target_class],threshold=0.1)
            return affordance,self.visualize_affordance(affordance)
        else:
            obstacle_affordance = self.get_obstacle_affordance()
            semantic_affordance = self.get_semantic_affordance([target_class],threshold=1.5)
            action_affordance = self.get_action_affordance(action)
            gpt4v_affordance = self.get_gpt4v_affordance(gpt4v_pcd)
            history_affordance = self.get_trajectory_affordance()
            affordance = 0.25*semantic_affordance + 0.25*action_affordance + 0.25*gpt4v_affordance + 0.25*history_affordance
            affordance = np.clip(affordance,0.1,1.0)
            affordance[obstacle_affordance == 0] = 0
            return affordance,self.visualize_affordance(affordance/(affordance.max()+1e-6))

    def get_debug_affordance_map(self,action,target_class,gpt4v_pcd):
        obstacle_affordance = self.get_obstacle_affordance()
        semantic_affordance = self.get_semantic_affordance([target_class],threshold=1.5)
        action_affordance = self.get_action_affordance(action)
        gpt4v_affordance = self.get_gpt4v_affordance(gpt4v_pcd)
        history_affordance = self.get_trajectory_affordance()
        return self.visualize_affordance(semantic_affordance/(semantic_affordance.max()+1e-6)),\
               self.visualize_affordance(history_affordance/(history_affordance.max()+1e-6)),\
               self.visualize_affordance(action_affordance/(action_affordance.max()+1e-6)),\
               self.visualize_affordance(gpt4v_affordance/(gpt4v_affordance.max()+1e-6)),\
               self.visualize_affordance(obstacle_affordance/(obstacle_affordance.max()+1e-6))

    def visualize_affordance(self,affordance):
        cmap = colormaps.get('jet')
        color_affordance = cmap(affordance)[:,0:3]
        color_affordance = cpu_pointcloud_from_array(self.navigable_pcd.point.positions.cpu().numpy(),color_affordance)
        return color_affordance
    
    def get_appeared_objects(self):
        return [entity['class'] for entity in self.object_entities]
    def get_2d_occupancy_map(self, grid_resolution=None):
        """
        Generate a 2D occupancy grid map from the point clouds.
        
        Returns:
            occupancy_map: 2D numpy array where:
                - 0 = unknown/unexplored
                - 1 = navigable/free space
                - 2 = obstacle
            map_info: dict with 'min_bound', 'max_bound', 'resolution', 'robot_position'
        """
        if grid_resolution is None:
            grid_resolution = self.grid_resolution
            
        try:
            # Get all relevant points
            navigable_points = self.navigable_pcd.point.positions.cpu().numpy()
            obstacle_points = self.obstacle_pcd.point.positions.cpu().numpy()
            
            # Combine to get bounds
            all_points = np.vstack([navigable_points, obstacle_points])
            min_bound = np.min(all_points, axis=0)
            max_bound = np.max(all_points, axis=0)
            
            # Create 2D grid (X-Y plane, ignoring Z/height)
            grid_size = np.ceil((max_bound[:2] - min_bound[:2]) / grid_resolution).astype(int)
            occupancy_map = np.zeros(grid_size, dtype=np.uint8)  # 0 = unknown
            
            # Mark navigable areas (value = 1)
            nav_indices = np.floor((navigable_points[:, :2] - min_bound[:2]) / grid_resolution).astype(int)
            nav_indices[:, 0] = np.clip(nav_indices[:, 0], 0, grid_size[0] - 1)
            nav_indices[:, 1] = np.clip(nav_indices[:, 1], 0, grid_size[1] - 1)
            occupancy_map[nav_indices[:, 0], nav_indices[:, 1]] = 1
            
            # Mark obstacles (value = 2)
            obs_indices = np.floor((obstacle_points[:, :2] - min_bound[:2]) / grid_resolution).astype(int)
            obs_indices[:, 0] = np.clip(obs_indices[:, 0], 0, grid_size[0] - 1)
            obs_indices[:, 1] = np.clip(obs_indices[:, 1], 0, grid_size[1] - 1)
            occupancy_map[obs_indices[:, 0], obs_indices[:, 1]] = 2
            
            # Robot position in grid coordinates
            robot_grid_pos = np.floor((self.current_position[:2] - min_bound[:2]) / grid_resolution).astype(int)
            
            map_info = {
                'min_bound': min_bound[:2],
                'max_bound': max_bound[:2],
                'resolution': grid_resolution,
                'robot_position': robot_grid_pos,
                'robot_world_position': self.current_position[:2]
            }
            
            return occupancy_map, map_info
        except Exception as e:
            print(f"[Mapper] Failed to generate 2D occupancy map: {e}")
            return None, None

    def save_2d_map_image(self, path="./", prefix="map"):
        """
        Save 2D map visualizations as images.
        
        Saves:
            - {prefix}_occupancy.png: Occupancy grid (white=free, black=obstacle, gray=unknown)
            - {prefix}_topdown.png: Top-down color view from point cloud
            - {prefix}_trajectory.png: Occupancy with robot trajectory overlay
        """
        import cv2
        
        try:
            occupancy_map, map_info = self.get_2d_occupancy_map()
            if occupancy_map is None:
                print("[Mapper] Cannot save 2D map - no data available")
                return
            
            # === 1. Occupancy Grid Image ===
            # 0=unknown (gray), 1=free (white), 2=obstacle (black)
            occ_image = np.zeros((*occupancy_map.shape, 3), dtype=np.uint8)
            occ_image[occupancy_map == 0] = [128, 128, 128]  # Gray - unknown
            occ_image[occupancy_map == 1] = [255, 255, 255]  # White - free
            occ_image[occupancy_map == 2] = [0, 0, 0]        # Black - obstacle
            
            # Mark robot position (red dot)
            robot_pos = map_info['robot_position']
            if 0 <= robot_pos[0] < occ_image.shape[0] and 0 <= robot_pos[1] < occ_image.shape[1]:
                cv2.circle(occ_image, (robot_pos[1], robot_pos[0]), 3, (0, 0, 255), -1)
            
            # Flip for proper orientation and scale up
            occ_image = cv2.flip(occ_image, 0)
            occ_image = cv2.resize(occ_image, (0, 0), fx=5, fy=5, interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(path + f"{prefix}_occupancy.png", occ_image)
            
            # === 2. Top-down Color Map ===
            try:
                scene_points = self.useful_pcd.point.positions.cpu().numpy()
                scene_colors = self.useful_pcd.point.colors.cpu().numpy()
                
                min_bound = map_info['min_bound']
                resolution = map_info['resolution']
                grid_size = occupancy_map.shape
                
                # Create color image
                color_map = np.zeros((*grid_size, 3), dtype=np.float32)
                color_count = np.zeros(grid_size, dtype=np.float32)
                
                indices = np.floor((scene_points[:, :2] - min_bound) / resolution).astype(int)
                indices[:, 0] = np.clip(indices[:, 0], 0, grid_size[0] - 1)
                indices[:, 1] = np.clip(indices[:, 1], 0, grid_size[1] - 1)
                
                for i, (idx, color) in enumerate(zip(indices, scene_colors)):
                    color_map[idx[0], idx[1]] += color
                    color_count[idx[0], idx[1]] += 1
                
                # Average colors
                color_count[color_count == 0] = 1
                color_map = color_map / color_count[:, :, np.newaxis]
                color_map = (color_map * 255).astype(np.uint8)
                color_map = cv2.flip(color_map, 0)
                color_map = cv2.resize(color_map, (0, 0), fx=5, fy=5, interpolation=cv2.INTER_NEAREST)
                cv2.imwrite(path + f"{prefix}_topdown.png", cv2.cvtColor(color_map, cv2.COLOR_RGB2BGR))
            except Exception as e:
                print(f"[Mapper] Failed to save top-down color map: {e}")
            
            # === 3. Trajectory Map ===
            try:
                traj_image = occ_image.copy()
                if len(self.trajectory_position) > 1:
                    trajectory = np.array(self.trajectory_position)
                    traj_indices = np.floor((trajectory[:, :2] - min_bound) / resolution).astype(int)
                    
                    # Scale up trajectory points
                    traj_indices = traj_indices * 5  # Match the 5x scale
                    
                    # Draw trajectory line
                    for i in range(len(traj_indices) - 1):
                        pt1 = (traj_indices[i, 1], occ_image.shape[0] - 1 - traj_indices[i, 0])
                        pt2 = (traj_indices[i + 1, 1], occ_image.shape[0] - 1 - traj_indices[i + 1, 0])
                        cv2.line(traj_image, pt1, pt2, (0, 255, 0), 2)
                    
                    # Mark start (blue) and current (red)
                    start_pt = (traj_indices[0, 1], occ_image.shape[0] - 1 - traj_indices[0, 0])
                    end_pt = (traj_indices[-1, 1], occ_image.shape[0] - 1 - traj_indices[-1, 0])
                    cv2.circle(traj_image, start_pt, 5, (255, 0, 0), -1)
                    cv2.circle(traj_image, end_pt, 5, (0, 0, 255), -1)
                
                cv2.imwrite(path + f"{prefix}_trajectory.png", traj_image)
            except Exception as e:
                print(f"[Mapper] Failed to save trajectory map: {e}")
            
            print(f"[Mapper] 2D maps saved to {path}")
            print(f"  - {prefix}_occupancy.png: Occupancy grid")
            print(f"  - {prefix}_topdown.png: Top-down color view")
            print(f"  - {prefix}_trajectory.png: Trajectory overlay")
            
        except Exception as e:
            print(f"[Mapper] Failed to save 2D map images: {e}")
    def save_pointcloud_debug(self,path="./"):
        save_pcd = o3d.geometry.PointCloud()
        try:
            assert self.useful_pcd.point.positions.shape[0] > 0
            save_pcd.points = o3d.utility.Vector3dVector(self.useful_pcd.point.positions.cpu().numpy())
            save_pcd.colors = o3d.utility.Vector3dVector(self.useful_pcd.point.colors.cpu().numpy())
            o3d.io.write_point_cloud(path + "scene.ply",save_pcd)
        except:
            pass
        try:
            assert self.navigable_pcd.point.positions.shape[0] > 0
            save_pcd.points = o3d.utility.Vector3dVector(self.navigable_pcd.point.positions.cpu().numpy())
            save_pcd.colors = o3d.utility.Vector3dVector(self.navigable_pcd.point.colors.cpu().numpy())
            o3d.io.write_point_cloud(path + "navigable.ply",save_pcd)
        except:
            pass
        try:
            assert self.obstacle_pcd.point.positions.shape[0] > 0
            save_pcd.points = o3d.utility.Vector3dVector(self.obstacle_pcd.point.positions.cpu().numpy())
            save_pcd.colors = o3d.utility.Vector3dVector(self.obstacle_pcd.point.colors.cpu().numpy())
            o3d.io.write_point_cloud(path + "obstacle.ply",save_pcd)
        except:
            pass
        
        object_pcd = o3d.geometry.PointCloud()
        for entity in self.object_entities:
            points = entity['pcd'].point.positions.cpu().numpy()
            colors = entity['pcd'].point.colors.cpu().numpy()
            new_pcd = o3d.geometry.PointCloud()
            new_pcd.points = o3d.utility.Vector3dVector(points)
            new_pcd.colors = o3d.utility.Vector3dVector(colors)
            object_pcd = object_pcd + new_pcd
        if len(object_pcd.points) > 0:
            o3d.io.write_point_cloud(path + "object.ply",object_pcd)
    
   
