"""
This is all about the datasets reading/loading from KITTI datasets in local machine
(KITTIReader is inherited from base dataset)
"""
from __future__ import absolute_import, division
import numpy as np
import os
from dataset_base import BaseDataReader
from utility import *
import sys

ros_cv2 = "/opt/ros/kinetic/lib/python2.7/dist-packages"
if ros_cv2 in sys.path:
    sys.path.remove(ros_cv2)
import cv2
import multiprocessing
import pdb
from kitti_config import cfg
from show_utility import *
from pyqtgraph.Qt import QtGui

import sys
sys.path.insert(0, "/mnt/raid1/Research/all-utility/")
from transform_utility import point_transform

class KITTIReader(BaseDataReader):
    """
    This is KITTI reader class, which contains KITTIObjectsReader and KITTITrackingReader
    """
    def getDatasets(self):
        print ("")

    def _getLabelData(self, values):
        label_data = {
                'type':         values[0],                      # 'Car', 'Pedestrian'
                'truncated':    float(values[1]),               # truncated pixel ratio [0..1]
                'occluded':     int(values[2]),                 # 0 = visible, 1 = partly occluded, 2 = fully occluded, 3 = unknown
                'alpha':        float(values[3]),               # object observation angle [-pi, pi]
                '2D_bbox':      {
                                    'left':  float(values[4]),
                                    'top':   float(values[5]),
                                    'right': float(values[6]),
                                    'bottom':float(values[7])
                                },
                '3D_dimensions':{
                                    'height':float(values[8]),
                                    'width': float(values[9]),
                                    'length':float(values[10])
                                },
                '3D_location':  {
                                    'x': float(values[11]),
                                    'y': float(values[12]),
                                    'z': float(values[13])
                                },
                'rotation_y' : float(values[14]),
                }

        return label_data


    def _processLabel(self, kitti_label):
        """
        Transform KITTI label to universal format
        """
        label = {
                'category': kitti_label['type'].lower(),
                'bbox2D':   kitti_label['2D_bbox'].copy(),
                'bbox3D':   {
                                    'location': {
                                                'x': kitti_label['3D_location']['x'],
                                                'y': kitti_label['3D_location']['y'] - kitti_label['3D_dimensions']['height'] / 2.0,
                                                'z': kitti_label['3D_location']['z'],
                                        },
                                    'dimensions': kitti_label['3D_dimensions'].copy(),
                                    'rotation_y': kitti_label['rotation_y'],

                                },
                'info':     {
                                    'truncated': kitti_label['truncated'],
                                    'occluded':  kitti_label['occluded'],
                                }
                }
        if 'trackID' in kitti_label:
            label['info']['trackID'] = kitti_label['trackID']

        return label


    def _getImageDirs(self, dataset=None):
        raise NotImplementedError("_getImageDirs() is not implemented in KITTIReader")

    def _getCamCalibration(self, frameID, dataset=None):
        raise NotImplementedError("_getCamCalibration() is not implemented in KITTIReader")

    def getFrameInfo(self, frameID, dataset=None):
        img_dir_left, img_dir_right = self._getImageDirs(dataset)
        img_file_left = os.path.join(img_dir_left, "%06d.png" % frameID)
        img_file_right = os.path.join(img_dir_right, "%06d.png" % frameID)
        calibration = self._getCamCalibration(frameID, dataset)

        return {
                'dataset': dataset,
                'frameID': frameID,
                'image_left': cv2.imread(img_file_left) if os.path.isfile(img_file_left) else None,
                'image_right': cv2.imread(img_file_right) if os.path.isfile(img_file_right) else None,
                'calibration': calibration,
                'lidar': self._getLidarPoints(frameID, dataset),
                'labels': self._getFrameLabels(frameID, dataset),
                }

           
    def _getLidarPoints(self, frameID, dataset=None):
        filename = os.path.join(self._getLidarDir(dataset), '%06d.bin' % frameID)
        if not os.path.isfile(filename):
            return None
        data = np.fromfile(filename, np.float32).reshape(-1, 4)
        return data


    def _getLidarDir(self, dataset=None):
        raise NotImplementedError("_getLidarDir() is not implemented in KITTIReader")

    def _readCamCalibration(self, filename):
        def line2values(line):
            return [float(v) for v in line.strip().split(" ")[1:]]
        def getMatrix(values, shape):
            return np.matrix(values, dtype=np.float32).reshape(shape)
        def padMatrix(matrix_raw):
            matrix = np.matrix(np.zeros((4, 4), dtype=np.float32), copy=False)
            matrix[:matrix_raw.shape[0], :matrix_raw.shape[1]] = matrix_raw
            matrix[3, 3] = 1
            return matrix

        with open(filename, 'r') as f:
            data = f.read().split("\n")

        P2 = getMatrix(line2values(data[2]), (3, 4))
        P3 = getMatrix(line2values(data[3]), (3, 4))

        Rect = padMatrix(getMatrix(line2values(data[4]), (3, 3)))
        velo2cam = padMatrix(getMatrix(line2values(data[5]), (3, 4)))

        P_left = P2
        P_right = P3

        f = P_left[0, 0]
        Tx = (P_right[0, 3] - P_left[0, 3]) / f
        cx_left = P_left[0, 2]
        cx_right = P_right[0, 2]
        cy = P_left[1, 2]

        reprojection = np.matrix([
                            [1, 0, 0, -cx_left],
                            [0, 1, 0, -cy],
                            [0, 0, 0, f],
                            [0, 0, -1/Tx, (cx_left - cx_right) / Tx],
                            ], dtype = np.float32)

        result = {
                'projection_left': P_left,
                'projection_right': P_right,
                'rect': Rect,
                'velo2cam': velo2cam,
                'reprojection': reprojection,
                }
        return result

    def volume_worker(self, filelist):
        for frame_id in filelist:
            velo = self._getLidarPoints(frame_id)
            avail_idx = np.logical_and(
                            np.logical_and(
                                np.logical_and( velo[:, 0] >= cfg.left, velo[:, 0] <= cfg.right),
                                np.logical_and( velo[:, 1] >= cfg.bottom, velo[:, 1] <= cfg.top))
                                    , np.logical_and( velo[:, 2] >= cfg.low, velo[:, 2] <= cfg.high))
            velo = velo[avail_idx, :]
           
    def camera_to_lidar(self, x, y, z):
        p = np.array([x, y, z, 1])
        p = np.matmul(np.linalg.inv(np.array(cfg.MATRIX_R_RECT_0)), p)
        p = np.matmul(np.linalg.inv(np.array(cfg.MATRIX_T_VELO_2_CAM)), p)
        p = p[:3]
        return tuple(p)

    def gt_worker(self, filelist):
        """
        Generate the ground truth
        """
#        interested_objects = ["Car", "Pedestrian", "Cyclist", "Van", "Truck"]
    
        for frame_id in filelist:
            # The following works
            boxMatrix = self.generateBoxMatrix(frame_id, "lidar")
            calibration = self._getCamCalibration(frame_id)
            f_label = '/mnt/raid1/data/kitti/training/label_2/' + "%06d.txt" % frame_id
            label = []
            label = [line for line in open(f_label, 'r').readlines()]
            gt_boxes3d = self.label2GtBox3d(label, "Car", "lidar", calibration)

            print(boxMatrix, gt_boxes3d)
    
    def label2GtBox3d(self, label, cls='Car', coordinate='camera', calibration=None):
        # Input:
        #   label: list (N)
        #   cls: 'Car' or 'Pedestrain' or 'Cyclist'
        #   coordinate: 'camera' or 'lidar'
        # Output:
        #   (N, 7)
        boxes3d = []
        if cls == 'Car':
            acc_cls = ['Car', 'Van']
        elif cls == 'Pedestrian':
            acc_cls = ['Pedestrian']
        elif cls == 'Cyclist':
            acc_cls = ['Cyclist']
        else: # all
            acc_cls = []

        boxes3d = []
        for line in label:
            ret = line.split()
            if ret[0] in acc_cls or acc_cls == []:
                h, w, l, x, y, z, r = [float(i) for i in ret[-7:]]
                boxes3d.append([x, y-h/2, z, l, w, h, r])

        boxes3d = np.array(boxes3d)
        if boxes3d.shape[0] and coordinate.lower() == "lidar":
            boxes3d[:, :3] = self.cameraToLidar(boxes3d[:, :3], calibration['velo2cam'], calibration['rect'])
            boxes3d[:, -1] = -boxes3d[:, -1] - np.pi / 2
        return boxes3d
         
    def angleInLimit(self, angle):
        # To limit the angle in -pi/2 - pi/2
        limit_degree = 5
        while angle >= np.pi / 2:
            angle -= np.pi
        while angle < -np.pi / 2:
            angle += np.pi
        if abs(angle + np.pi / 2) < limit_degree / 180 * np.pi:
            angle = np.pi / 2
        return angle

    def convertLabelToGroundTruth(self):
        """
        Convert from the label_2 in Camera-coord to 3D bounding box in Lidar-coord
        """
        frame_id_list = [f for f in range(7481)]
        num_worker = 40
        ROI = [cfg.left, cfg.bottom, cfg.low, cfg.right, cfg.top, cfg.high]
        for sublist in np.array_split(frame_id_list, num_worker):
            p = multiprocessing.Process(target = self.gt_worker, args=(sublist, ))
            p.start()
#        filelist = [os.path.splitext(f)[0] for f in os.listdir(self._getLidarDir()) if f.endswith('.bin')]


    def cameraToLidar(self, points_in_cam, Tr_velo_to_cam=None, R0_rect=None):
        """
        Convert points in camera coordinate to points in Lidar coordinate
        """
        N = points_in_cam.shape[0]
        points = np.hstack([points_in_cam, np.ones((N, 1))]).T
        if type(Tr_velo_to_cam) == type(None):
            Tr_velo_to_cam = np.array(cfg.MATRIX_T_VELO_2_CAM)

        if type(R0_rect) == type(None):
            R0_rect = np.array(cfg.MATRIX_R_RECT_0)

        points = np.matmul(np.linalg.inv(R0_rect), points)
        points = np.matmul(np.linalg.inv(Tr_velo_to_cam), points).T
        points = points[:, 0:3]
        return points


    def lidarToCamera(self, points, Tr_velo_to_cam=None, R0_rect=None):
        """
        Conver points in Lidar coordinate to points in cam coordinate
        """
        # (N, 3) -> (N, 3)
        N = points.shape[0]
        points = np.hstack([points, np.ones((N, 1))]).T
        points = np.matmul(Tr_velo_to_cam, points)
        points = np.matmul(R0_rect, points).T
        points = points[:, 0:3]
        return np.asarray(points.reshape(-1, 3), dtype=np.float32)


    def generateBoxMatrix(self, frame_id, coor = "camera"):
        """
        make a 3D box from label in camera
        If generate 3d bbox in lidar coordinate, make sure perform the following 2 steps: 
        1, convert location (center point) from camera into lidar
        2, yaw_angle = -yaw_angle - np.pi / 2

        """
        calibration = kitti._getCamCalibration(frame_id)
        label = kitti._getFrameLabels(frame_id)
#        interested_list = ["car", "van"]
        interested_list = ["car"]
        idx_gt =  [counter for counter, value in enumerate(label) if value['category'] in interested_list]
        label_matrix = np.zeros((len(idx_gt), 8))

        for i in range(len(idx_gt)):
            label_matrix[i, 0] = label[idx_gt[i]]['bbox3D']['location']['x'] 
            label_matrix[i, 1] = label[idx_gt[i]]['bbox3D']['location']['y'] 
            label_matrix[i, 2] = label[idx_gt[i]]['bbox3D']['location']['z'] 
            label_matrix[i, 3] = label[idx_gt[i]]['bbox3D']['dimensions']['length'] 
            label_matrix[i, 4] = label[idx_gt[i]]['bbox3D']['dimensions']['width'] 
            label_matrix[i, 5] = label[idx_gt[i]]['bbox3D']['dimensions']['height']

            label_matrix[i, 6] = label[idx_gt[i]]['bbox3D']['rotation_y']
            if label[idx_gt[i]]['category'] == "car" or label[idx_gt[i]]['category'] == "van":
                label_matrix[i, 7] = 1.0
            elif label[idx_gt[i]]['category'] == "pedestrian":
                label_matrix[i, 7] = 2.0
            elif label[idx_gt[i]]['category'] == "cyclist":
                label_matrix[i, 7] = 3.0

        if coor.lower() == "lidar":
            label_matrix[:, :3] = kitti.cameraToLidar(label_matrix[:, :3],
                                 calibration['velo2cam'], calibration['rect'])    


            label_matrix[:, -2] *= -1
            label_matrix[:, -2] -= np.pi / 2
           
            for i in range(label_matrix.shape[0]):
                if label_matrix[i, -2] < -np.pi / 2:
                    label_matrix[i, -2] += np.pi

        return label_matrix

class KITTIObjectsReader(KITTIReader):
    """
    Class for KITTI object detection data reader
    """
    def getDatasets(self):
        return ['None']

    def _getLabelsDir(self):
        label_dir = os.path.join(self._dir, 'label_2')
        if os.path.exists(label_dir):
            return label_dir
        return None

    def _getFrameLabels(self, frameID, dataset=None):
        if self._getLabelsDir() is None:
            return []
        else:
            with open(os.path.join(self._getLabelsDir(), "%06d.txt" % frameID), 'r') as f:
                text_data = [[value for value in line.split(" ")] for line in f.read().split('\n') if line]
            labels = []
            for line in text_data:
                label_data = self._getLabelData(line)
                labels.append(self._processLabel(label_data))

            return labels
    
    def _get3DBoxDirs(self, dataset=None):
        return (os.path.join(self._dir, "velodyne_3dbbox_new"))

    def _getImageDirs(self, dataset=None):
        return (os.path.join(self._dir, "image_2"), os.path.join(self._dir, "image_3"))

    def _getCalibrationDir(self):
        return os.path.join(self._dir, 'calib')

    def _getCamCalibration(self, frameID, dataset=None):
        return self._readCamCalibration(os.path.join(self._getCalibrationDir(), "%06d.txt" % frameID))

    def _getLidarDir(self, dataset=None):
        return os.path.join(self._dir, 'velodyne')

    def getGFrameInfo(self, frameID, dataset=None):
        print("Nothing")

class KITTITrackingReader(KITTIReader):
    def _getLidarDir(self, tracking_sequence_num = 0, dataset=None):
        return os.path.join(self._dir, 'velodyne', '%04d'%tracking_sequence_num)


def generateBoxMatrix(frame_id, kitti, coor="lidar"):
    """
    make a 3D box from label in camera
    If generate 3d bbox in lidar coordinate, make sure perform the following 2 steps: 
    1, convert location (center point) from camera into lidar
    2, yaw_angle = -yaw_angle - np.pi / 2

    """
    calibration = kitti._getCamCalibration(frame_id)

    with open(os.path.join("/mnt/raid1/Research/VoxelNet/voxelnet/predictions/123/data", "%06d.txt" % frame_id), 'r') as f:
        text_data = [[value for value in line.split(" ")] for line in f.read().split('\n') if line]

    idx_gt = 0
    for i in range(len(text_data)):
        if text_data[i][0] == 'Car':
            idx_gt += 1
        
    label_matrix = np.zeros((idx_gt, 7))
    for i in range(idx_gt):
        label_matrix[i, 0] = float(text_data[i][-5])
        label_matrix[i, 1] = float(text_data[i][-4]) - float(text_data[i][-8]) / 2. 
        label_matrix[i, 2] = float(text_data[i][-3])
        label_matrix[i, 3] = float(text_data[i][-6])
        label_matrix[i, 4] = float(text_data[i][-7])
        label_matrix[i, 5] = float(text_data[i][-8])
        label_matrix[i, 6] = float(text_data[i][-2])

    if coor.lower() == "lidar":
        label_matrix[:, :3] = kitti.cameraToLidar(label_matrix[:, :3],
                             calibration['velo2cam'], calibration['rect'])    
        label_matrix[:, -1] *= -1
        label_matrix[:, -1] -= np.pi / 2
        
    return label_matrix


def label_to_gt_box3d(labels, cls='Car', coordinate='camera'):
    # Input:
    #   label: (N, N')
    #   cls: 'Car' or 'Pedestrain' or 'Cyclist'
    #   coordinate: 'camera' or 'lidar'
    # Output:
    #   (N, N', 7)
    boxes3d = []
    if cls == 'Car':
        acc_cls = ['Car', 'Van']
    elif cls == 'Pedestrian':
        acc_cls = ['Pedestrian']
    elif cls == 'Cyclist':
        acc_cls = ['Cyclist']
    else: # all
        acc_cls = []

    for label in labels:
        boxes3d_a_label = []
        for line in label:
            ret = line.split()
            if ret[0] in acc_cls or acc_cls == []:
                h, w, l, x, y, z, r = [float(i) for i in ret[-7:]]
                box3d = np.array([x, y, z, h, w, l, r])
                boxes3d_a_label.append(box3d)
        if coordinate == 'lidar':
            boxes3d_a_label = camera_to_lidar_box(np.array(boxes3d_a_label))

        boxes3d.append(np.array(boxes3d_a_label).reshape(-1, 7))
    return boxes3d

def camera_to_lidar_box(boxes):
    # (N, 7) -> (N, 7) x,y,z,h,w,l,r
    ret = []
    for box in boxes:
        x, y, z, h, w, l, ry = box
        (x, y, z), h, w, l, rz = camera_to_lidar(
            x, y, z), h, w, l, -ry - np.pi / 2
        rz = angle_in_limit(rz)
        ret.append([x, y, z, h, w, l, rz])
    return np.array(ret).reshape(-1, 7)

def angle_in_limit(angle):
    # To limit the angle in -pi/2 - pi/2
    limit_degree = 5
    while angle >= np.pi / 2:
        angle -= np.pi
    while angle < -np.pi / 2:
        angle += np.pi
    if abs(angle + np.pi / 2) < limit_degree / 180 * np.pi:
        angle = np.pi / 2
    return angle

def generateTightBox():
    f_label = '/mnt/raid1/data/kitti/training/label_2/000008.txt'
    label = []
    label.append([line for line in open(f_label, 'r').readlines()])
    batch_gt_boxes3d = label_to_gt_box3d(label, cls="Car", coordinate="lidar")
    print(batch_gt_boxes3d)


if __name__ == "__main__":
    data_dir = '/mnt/raid1/data/kitti/training'
    #data_dir = '/mnt/raid1/Research/VoxelNet/voxelnet/data/training_data/validation'
    #data_dir = '/mnt/raid1/Research/VoxelNet/voxelnet/data/training_data/verify_data_augmentation'
    kitti = KITTIObjectsReader(data_dir)
    #kitti.convertLabelToGroundTruth()
    #prediction_matrix = generateBoxMatrix(frame_id, kitti)
    frame_id = int(sys.argv[1])
    calibration = kitti._getCamCalibration(frame_id)
    pc = kitti._getLidarPoints(frame_id)
    label_matrix_lidar = kitti.generateBoxMatrix(frame_id, "lidar")
    # rotate angle all in Lidar system
    angle = 0 
    if angle != 0.:
        pc[:, 0:3] = point_transform(pc[:, :3], 0, 0, 0, rz = angle) 
        label_matrix_lidar[:, :3] = point_transform(label_matrix_lidar[:, :3], 0, 0, 0, rz = angle)
        label_matrix_lidar[:, -2] = label_matrix_lidar[:, -2] - angle 


    show_in_lidar = False 

    if show_in_lidar:
        showPC(pc, label_matrix_lidar[:, :-1])

    else:
        # Test everything in cam:
        pc_in_cam = kitti.lidarToCamera(pc[:, :3], calibration['velo2cam'], calibration['rect'])
        label_matrix_cam = kitti.generateBoxMatrix(frame_id)
        print(label_matrix_cam)
        # Rotate point cloud and labels with an angle in cam system
        #angle_in_cam = -1.57 
        #pc_in_cam = point_transform(pc_in_cam[:, :3], 0, 0, 0, ry = angle_in_cam)
        #label_matrix_cam[:, :3] = point_transform(label_matrix_cam[:, :3], 0, 0, 0, ry = angle_in_cam)
        #label_matrix_cam[:, -2] = label_matrix_cam[:, -2] - angle_in_cam
        #for i in range(label_matrix_cam.shape[0]):
        #    if label_matrix_cam[i, -2] > np.pi:
        #        label_matrix_cam[i, -2] -= 2 * np.pi
        #    elif label_matrix_cam[i, -2] < -np.pi:
        #        label_matrix_cam[i, -2] += 2 * np.pi


        #showPC_inCam(pc_in_cam, label_matrix_cam[:, :-1])

        # If we want to show the detection in single frame
        detect_matrix_cam = np.loadtxt("/home/saiclei/%06d.txt" % frame_id)
        detect_matrix_cam = detect_matrix_cam[:, [0,1,2,3,4,5,6]]
        showPC_inCam(pc_in_cam, label_matrix_cam[:, :-1], detect_matrix_cam) 
