import carla
import weakref
import math
import cv2
import numpy as np
import open3d as o3d
from matplotlib import cm
from CommonLibs.UniversalLib import get_boundingbox
from Localization.LocalizationManager import vehicle_loc

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255),  # None
    (70, 70, 70),  # Building
    (100, 40, 40),  # Fences
    (55, 90, 80),  # Other
    (220, 20, 60),  # Pedestrian
    (153, 153, 153),  # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),  # Vehicle
    (102, 102, 156),  # Wall
    (220, 220, 0),  # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),  # Ground
    (150, 100, 100),  # Bridge
    (230, 150, 140),  # RailTrack
    (180, 165, 180),  # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160),  # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),  # Water
    (145, 170, 100),  # Terrain
]) / 255.0  # normalize each channel [0-1] since is what Open3D uses

class SemanticLidarSensor:
    """
    Semantic lidar sensor manager. 

    Parameters
    ----------
    vehicle : carla.Vehicle object
    world : carla.World object
    sensor : carla.sensor, Lidar sensor that will be attached to the vehicle.


    """

    def __init__(self, vehicle, world):

        #world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')

        # set attribute based on the configuration
        blueprint.set_attribute('upper_fov', '2')
        blueprint.set_attribute('lower_fov', '-25')
        blueprint.set_attribute('channels','32')
        blueprint.set_attribute('range', '120')
        blueprint.set_attribute(
            'points_per_second', '1000000')
        blueprint.set_attribute(
            'rotation_frequency', '200')

        # spawn sensor
        spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.9))
        self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
        
        # lidar data
        self.points = None
        self.obj_idx = None
        self.obj_tag = None

        self.timestamp = None
        self.frame = 0
        # open3d point cloud object
        self.o3d_pointcloud = o3d.geometry.PointCloud()

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: SemanticLidarSensor._on_data_event(
                weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        """Semantic Lidar  method"""
        self = weak_self()
        if not self:
            return

        # shape:(n, 6)
        data = np.frombuffer(event.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('CosAngle', np.float32), ('ObjIdx', np.uint32),
            ('ObjTag', np.uint32)]))

        # (x, y, z, intensity)
        self.points = np.array([data['x'], data['y'], data['z']]).T
        self.obj_tag = np.array(data['ObjTag'])
        self.obj_idx = np.array(data['ObjIdx'])
        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp
        
    def destroy(self):
        self.sensor.destroy()

    def filter_vehicle_out_sensor(self,vehicle_list):
            """
            filter vehicles which is out of lidar range
            
            Parameters
            ----------
            vehicle_list : list
                The list contains all vehicles information retrieves from the
                server.

            Returns
            -------
            new_vehicle_list : list
                The list that filters out the out of scope vehicles.
            """
            
            semantic_idx = self.obj_idx
            semantic_tag = self.obj_tag
            
            # label 10 is the vehicle
            vehicle_idx = semantic_idx[semantic_tag == 10]
            # each individual instance id
            vehicle_unique_id = list(np.unique(vehicle_idx))
            #print(vehicle_unique_id)
            new_vehicle_list = []
            for veh in vehicle_list:
                if veh.id in vehicle_unique_id:
                    new_vehicle_list.append(veh)
            return new_vehicle_list

    def get_nearby(self,vehicles,map):
        #get all the info of target vehicles, including motion and position and lane info
        ego_vehicle=vehicles[0]
        transform_o=ego_vehicle.get_transform()
        if len(vehicles) > 1:
                    udp_list=[]
                    vehicles_new=self.filter_vehicle_out_sensor(vehicles)
                    distance = lambda l: math.sqrt((l.x - transform_o.location.x)**2 + (l.y - transform_o.location.y)**2 + (l.z - transform_o.location.z)**2)
                    vehicles = [(distance(vehicle.get_location()), vehicle) for vehicle in vehicles_new if vehicle.id != ego_vehicle.id]
                    for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                        if d > 120.0:
                            break
                        else:   
                            udp_list_1,waypoint_o=vehicle_loc(map,vehicle)               
                            for ii in udp_list_1:
                                udp_list.append(ii)
                            for ii in get_boundingbox(vehicle):
                                udp_list.append(ii)    
                            udp_list.append(vehicle.id)   
                    return udp_list
        else:
            return [0]
class CameraSensor:
    """
    Camera manager for vehicle or infrastructure, temporay

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle, this is for cav.

    world : carla.World
        The carla world object, this is for rsu.

    relative_position : str
        Indicates the sensor is a front or rear camera. option:
        front, left, right.

    Attributes
    ----------
    image : np.ndarray
        Current received rgb image.
    sensor : carla.sensor
        The carla sensor that mounts at the vehicle.

    """

    def __init__(self, vehicle, world, relative_position):
        if vehicle is not None:
            world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('fov', '100')

        spawn_point = self.spawn_point_estimation(relative_position)

        if vehicle is not None:
            self.sensor = world.spawn_actor(
                blueprint, spawn_point, attach_to=vehicle)
        else:
            self.sensor = world.spawn_actor(blueprint, spawn_point)

        self.image = None
        self.timstamp = None
        self.frame = 0
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CameraSensor._on_rgb_image_event(
                weak_self, event))

        # camera attributes
        self.image_width = int(self.sensor.attributes['image_size_x'])
        self.image_height = int(self.sensor.attributes['image_size_y'])

    @staticmethod
    def spawn_point_estimation(relative_position):

        pitch = 0
        carla_location = carla.Location(x=0, y=0, z=0)


        if relative_position == 'front':
            carla_location = carla.Location(x=carla_location.x + 2.5,
                                            y=carla_location.y,
                                            z=carla_location.z + 1.0)
            yaw = 0

        elif relative_position == 'right':
            carla_location = carla.Location(x=carla_location.x + 0.0,
                                            y=carla_location.y + 0.3,
                                            z=carla_location.z + 1.8)
            yaw = 100

        elif relative_position == 'left':
            carla_location = carla.Location(x=carla_location.x + 0.0,
                                            y=carla_location.y - 0.3,
                                            z=carla_location.z + 1.8)
            yaw = -100
        else:
            carla_location = carla.Location(x=carla_location.x - 2.0,
                                            y=carla_location.y,
                                            z=carla_location.z + 1.5)
            yaw = 180

        carla_rotation = carla.Rotation(roll=0, yaw=yaw, pitch=pitch)
        spawn_point = carla.Transform(carla_location, carla_rotation)

        return spawn_point

    @staticmethod
    def _on_rgb_image_event(weak_self, event):
        """CAMERA  method"""
        self = weak_self()
        if not self:
            return
        image = np.array(event.raw_data)
        image = image.reshape((self.image_height, self.image_width, 4))
        # we need to remove the alpha channel
        image = image[:, :, :3]

        self.image = image
        self.frame = event.frame
        self.timestamp = event.timestamp

    def camera_show(self):
        rgb_image = np.array(self.image)
        rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)
        cv2.imshow('Front Camera of Ego Vehicle', rgb_image)       # 建立名为‘CAR’ 的窗口并显示图像
        cv2.waitKey(1)       # waitKey代表读取键盘的输入，0代表一直等待
       
        
        


class LidarSensor:
    """
    Lidar sensor manager.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle, this is for cav.

    world : carla.World
        The carla world object, this is for rsu.



    Attributes
    ----------
    o3d_pointcloud : 03d object
        Received point cloud, saved in o3d.Pointcloud format.

    sensor : carla.sensor
        Lidar sensor that will be attached to the vehicle.

    """

    def __init__(self, vehicle, world):
        if vehicle is not None:
            world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')

        # set attribute based on the configuration
        blueprint.set_attribute('upper_fov', '2')
        blueprint.set_attribute('lower_fov', '-25')
        blueprint.set_attribute('channels', '32')
        blueprint.set_attribute('range', '120')
        blueprint.set_attribute(
            'points_per_second', '1000000')
        blueprint.set_attribute(
            'rotation_frequency', '20')
       

        # spawn sensor      
        spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.9))
        self.sensor = world.spawn_actor(blueprint, spawn_point)

        # lidar data
        self.data = None
        self.timestamp = None
        self.frame = 0
        # open3d point cloud object
        self.o3d_pointcloud = o3d.geometry.PointCloud()

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: LidarSensor._on_data_event(
                weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        """Lidar  method"""
        self = weak_self()
        if not self:
            return

        # retrieve the raw lidar data and reshape to (N, 4)
        data = np.copy(np.frombuffer(event.raw_data, dtype=np.dtype('f4')))
        # (x, y, z, intensity)
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp

class VisualizerManager:
    def __init__(self) :
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(
            window_name='Carla Lidar',
            width=960,
            height=540,
            left=480,
            top=270)
        self.vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        self.vis.get_render_option().point_size = 1
        self.vis.get_render_option().show_coordinate_frame = True
    def sematic_lidar_show(self,frame,sematic_lidar_object):
        point_list=sematic_lidar_object.o3d_pointcloud
        point_list.points=o3d.utility.Vector3dVector(sematic_lidar_object.points)
        point_list.colors = o3d.utility.Vector3dVector(LABEL_COLORS[sematic_lidar_object.obj_tag])
        if frame == 2:
            self.vis.add_geometry(point_list)
        self.vis.update_geometry(point_list)
        self.vis.poll_events()
        self.vis.update_renderer()
    def rgb_camera_show(self):
        pass#tbd
