import logging
from pathlib import Path

import numpy as np
import quaternion

from .mapping_utils.geometry import *
from .mapping_utils.preprocess import *
from .mapping_utils.projection import *
from .mapping_utils.transform import *
from .mapping_utils.path_planning import *
# from .cv_utils.image_percevior import GLEE_Percevior
from matplotlib import colormaps
from .constants import *
import open3d as o3d
from PIL import Image

class Mapper:
    def __init__(self,
                 camera_intrinsic=None,
                 pcd_resolution=0.05,
                 grid_resolution=0.1,
                 grid_size=5,
                 floor_height=-0.8,
                 ceiling_height=0.8,
                 translation_func=habitat_translation,
                 rotation_func=habitat_rotation,
                 rotate_axis=[0,1,0],
                 device='cpu:0',
                 enable_semantics=False,
                 debug=True,
                 depth_debug_dir=None,
                 depth_colormap="turbo"):
        self.device = device
        self.camera_intrinsic = np.array(camera_intrinsic, dtype=float) if camera_intrinsic is not None else None
        self.pcd_resolution = pcd_resolution
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.floor_height = floor_height
        self.ceiling_height = ceiling_height
        self.translation_func = translation_func if translation_func is not None else self._identity_translation
        self.rotation_func = rotation_func if rotation_func is not None else self._identity_rotation
        self.rotate_axis = np.array(rotate_axis)
        self.enable_semantics = enable_semantics
        self.object_percevior = GLEE_Percevior(device=device) if self.enable_semantics else None
        self.pcd_device = o3d.core.Device(device.upper())
        self.debug = debug
        self.depth_debug_dir = Path(depth_debug_dir) if depth_debug_dir else None
        if self.depth_debug_dir is None and self.debug:
            self.depth_debug_dir = Path("./outputs/depth_debug")
        if self.depth_debug_dir:
            self.depth_debug_dir.mkdir(parents=True, exist_ok=True)
        self.depth_colormap = depth_colormap
        self.update_iterations = 0
        self._logger = logging.getLogger(__name__)
    
    def reset(self,position,rotation):
        self._ensure_intrinsic()
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
        self._ensure_intrinsic()
        self.current_position = self.translation_func(position) - self.initial_position
        self.current_rotation = self.rotation_func(rotation)
        self.current_depth = preprocess_depth(depth)
        self.current_rgb = preprocess_image(rgb)
        self._log_depth_stats(self.current_depth)
        self._maybe_save_depth_image(self.current_depth)
        self.trajectory_position.append(self.current_position)
        # to avoid there is no valid depth value (especially in real-world)
        if np.sum(self.current_depth) > 0:
            camera_points,camera_colors = get_pointcloud_from_depth(self.current_rgb,self.current_depth,self.camera_intrinsic)
            world_points = translate_to_world(camera_points,self.current_position,self.current_rotation)
            self.current_pcd = gpu_pointcloud_from_array(world_points,camera_colors,self.pcd_device).voxel_down_sample(self.pcd_resolution)
            self._log_pcd_stats("current_pcd", self.current_pcd)
        else:
            return
        # semantic masking and project object mask to pointcloud
        if self.enable_semantics and self.object_percevior is not None:
            classes,masks,confidences,visualization = self.object_percevior.perceive(self.current_rgb)
            self.segmentation = visualization[0]
            current_object_entities = self.get_object_entities(self.current_depth,classes,masks,confidences)
            self.object_entities = self.associate_object_entities(self.object_entities,current_object_entities)
            self.object_pcd = self.update_object_pcd()
        else:
            self.segmentation = None
            current_object_entities = []
        # pointcloud update
        self.scene_pcd = gpu_merge_pointcloud(self.current_pcd,self.scene_pcd).voxel_down_sample(self.pcd_resolution)
        self._log_pcd_stats("scene_pcd", self.scene_pcd)
        self.scene_pcd = self.scene_pcd.select_by_index((self.scene_pcd.point.positions[:,2]>self.floor_height-0.2).nonzero()[0])
        self.useful_pcd = self.scene_pcd.select_by_index((self.scene_pcd.point.positions[:,2]<self.ceiling_height).nonzero()[0])
        
        # all the stairs will be regarded as navigable
        for entity in current_object_entities:
            if entity['class'] == 'stairs':
                self.navigable_pcd = gpu_merge_pointcloud(self.navigable_pcd,entity['pcd'])
        # geometry 
        current_navigable_point = self.current_pcd.select_by_index((self.current_pcd.point.positions[:,2]<self.floor_height).nonzero()[0])
        current_navigable_position = current_navigable_point.point.positions.cpu().numpy()
        if current_navigable_position.size == 0:
            self._logger.debug("No navigable points under floor_height; skip update.")
            return
        standing_position = np.array([self.current_position[0],self.current_position[1],current_navigable_position[:,2].mean()])
        interpolate_points = np.linspace(np.ones_like(current_navigable_position)*standing_position,current_navigable_position,25).reshape(-1,3)
        interpolate_points = interpolate_points[(interpolate_points[:,2] > self.floor_height-0.2) & (interpolate_points[:,2] < self.floor_height+0.2)]
        if interpolate_points.size == 0:
            self._logger.debug("Interpolated navigable points empty; skip update.")
            return
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

    def _identity_translation(self, position):
        return np.asarray(position, dtype=float)

    def _identity_rotation(self, rotation):
        try:
            quat = quaternion.quaternion(rotation[3], rotation[0], rotation[1], rotation[2])
            return quaternion.as_rotation_matrix(quat)
        except Exception:
            return np.eye(3, dtype=float)

    def _ensure_intrinsic(self):
        if self.camera_intrinsic is None:
            raise ValueError("camera_intrinsic is not set. Provide it when constructing Mapper.")

    def _log_depth_stats(self, depth):
        if not self.debug:
            return
        try:
            depth_np = np.asarray(depth)
            stats = {
                "shape": depth_np.shape,
                "dtype": str(depth_np.dtype),
                "min": float(np.nanmin(depth_np)),
                "max": float(np.nanmax(depth_np)),
                "mean": float(np.nanmean(depth_np)),
                "zeros": int(np.sum(depth_np == 0)),
                "nan": int(np.isnan(depth_np).sum()),
            }
            self._logger.info("[Mapper] Depth stats: %s", stats)
        except Exception:
            self._logger.info("[Mapper] Depth stats: failed to compute")
    
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
            point_colors = np.array([d3_40_colors_rgb[exist_objects.index(cls)%40]]*world_points.shape[0])
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

    def _maybe_save_depth_image(self, depth):
        if not self.debug or self.depth_debug_dir is None:
            return
        try:
            depth_np = np.asarray(depth, dtype=np.float32)
            valid_mask = depth_np > 0
            if valid_mask.any():
                depth_min = depth_np[valid_mask].min()
                depth_max = depth_np[valid_mask].max()
            else:
                depth_min = 0.0
                depth_max = 1.0
            span = max(depth_max - depth_min, 1e-6)
            normalized = np.clip((depth_np - depth_min) / span, 0.0, 1.0)
            cmap = colormaps.get(self.depth_colormap, colormaps.get('jet'))
            rgba = cmap(normalized)[..., :3]
            rgba[~valid_mask] = 0
            img = Image.fromarray((rgba * 255).astype(np.uint8))
            filename = self.depth_debug_dir / f"depth_{self.update_iterations:06d}.png"
            img.save(filename)
        except Exception as exc:
            self._logger.debug("[Mapper] Failed to save depth image: %s", exc)

    def _log_pcd_stats(self,label,pcd):
        if not self.debug:
            return
        try:
            points = pcd.point.positions.cpu().numpy()
            if points.size == 0:
                self._logger.info("[Mapper] %s empty", label)
                return
            mins = points.min(axis=0)
            maxs = points.max(axis=0)
            self._logger.info("[Mapper] %s points=%d range(min=%s max=%s)", label, points.shape[0], np.round(mins,3), np.round(maxs,3))
        except Exception:
            self._logger.debug("[Mapper] Failed logging stats for %s", label)
    
   
