"""
    File name: utility.py
    This utility file is designed to assist show Lidar and bbox more efficiently
    In particular, 
    1) if we are using SAIC npy point cloud, we prefer show everything in 
    lidar coordinate system.
    2) if we are using KITTI bin file, we prefer show everything in camera coordinate syste

    **Usage:
    python3 show_utility.py --use=show_lidar 
    --pc=/mnt/raid1/data/kitti/training/velodyne/000011.bin 
    --pl=/mnt/raid1/test/predictions/KITTI/000011.txt 
    --coor=camera 
    --gl=/mnt/raid1/data/kitti/training/label_2/000011.txt

    Author: Lei Yang
    Affiated: SAIC USA
"""
__author__ = 'Lei'
__copyright__ = 'Copyright 2018, SAIC Inc'
__credits__ = ['AD team']
__license__ = 'MIT'
__version__ = '0.0.1'
__maintainer__ = 'Lei'
__email__ = 'lyang@saicusa.com'
__status__ = 'Development'



import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import pdb
from mpl_toolkits.mplot3d import Axes3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui
import sys
import os

ros_cv2s = ["/opt/ros/kinetic/lib/python2.7/dist-packages",
            "/opt/ros/kinetic_pb/lib/python2.7/dist-packages"]
for ros_cv2 in ros_cv2s:
    if ros_cv2 in sys.path:
        sys.path.remove(ros_cv2)
import cv2


# Temporarily, using calib_utils function
avod_root = "/home/saiclei/github/Perception/avod_saic/"

sys.path.insert(0, avod_root)
sys.path.insert(0, os.path.join(avod_root, 'wavedata'))


from wavedata.tools.core import calib_utils
from wavedata.tools.obj_detection import obj_utils

import argparse
import sys

class MyGLView(gl.GLViewWidget):
    def __init__(self):
        super(MyGLView, self).__init__()

    def paintGL(self, *args, **kwds):
        gl.GLViewWidget.paintGL(self, *args, **kwds)
        self.qglColor(QtCore.Qt.white)
        self.renderText(0, 0, 0, 'Test')


class MyGLView_new(gl.GLViewWidget):
    def __init__(self, X, Y, Z, text):
        """
        X, Y, Z, text all should be text and have the same size
        """
        print(X.shape, Y.shape)
        len_X = len(X)
        if len_X == 0 or len(Y) != len_X or len(Z) != len_X or len(text) != len_X:
            raise ValueError("All X, Y, Z and text should have the same size")
        super(MyGLView, self).__init__()
        self.text = text
        self.X = X
        self.Y = Y
        self.Z = Z

    def setText(self, text):
        self.text = tet
        self.update()

    def setX(self, X):
        self.X = X
        self.update()

    def setY(self, Y):
        self.Y = Y
        self.update()

    def setZ(self, Z):
        self.Z = Z
        self.update()

    def paintGL(self, *args, **kwargs):
        gl.GLViewWidget.paintGL(self, *args, **kwargs)
        self.qglColor(QtCore.Qt.white)
        for i in range(len(self.X)):
            self.renderText(self.X[i], self.Y[i], self.Z[i], str(self.text[i]))

def showPCInCam(point_cloud, bbox3D_label=None, prediction_label=None):
    """
    Show point cloud using pyqtopengl,
    bbox3D_label is a ndarray(n, 7) --> (x, y, z, l, w, h, yaw_angle)
    """
    app = QtGui.QApplication([])
    pg.mkQApp()
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    view_widget = gl.GLViewWidget()
    """
    if bbox3D_label is not None:
        X = bbox3D_label[:, 0]
        Y = bbox3D_label[:, 1]
        Z = bbox3D_label[:, 2]
        text = bbox3D_label[:, -1]
        view_widget = MyGLView(X, Y, Z, text)
    else:
        view_widget = gl.GLViewWidget()
    """
    view_widget.show()
    view_widget.setWindowTitle('3D Lidar Show Tools')


    ax = gl.GLAxisItem()
    ax.setSize(60, 60, 60)
    view_widget.addItem(ax)

    #b = gl.GLBoxItem()
    #view_widget.addItem(b)


    xgrid = gl.GLGridItem()
    ygrid = gl.GLGridItem()
    zgrid = gl.GLGridItem()
    view_widget.addItem(xgrid)
    view_widget.addItem(ygrid)
    view_widget.addItem(zgrid)

    line = gl.GLLinePlotItem()
    view_widget.addItem(line)

    xgrid.rotate(90, 0, 1, 0)
    ygrid.rotate(90, 1, 0, 0)
    
    x = point_cloud[:, 0]
    y = point_cloud[:, 1]
    z = point_cloud[:, 2]
    scatter_plot = gl.GLScatterPlotItem(pos = point_cloud[:, :3], color = pg.glColor('g'), size = 0.5)
    view_widget.addItem(scatter_plot)
    if bbox3D_label is not None:
        #mkPen('y', width = 3, stype=QtCore.Qt.DashLine)
        #drawArrow(view_widget, bbox3D_label)
        bbox3D_corners = center_to_corner_box3d_cam(bbox3D_label)
        draw3DBox(view_widget, bbox3D_corners, color=pg.glColor('r'))
        #pts = getBoxPts(bbox3D_corners)
        
        #for i in range(pts.shape[0]):
        #    plt = gl.GLLinePlotItem(pos = pts[i], color = pg.glColor('r'))
        #    view_widget.addItem(plt)


    if prediction_label is not None:
        #mkPen('y', width = 3, stype=QtCore.Qt.DashLine)
        bbox3D_corners = center_to_corner_box3d_cam(prediction_label)
        draw3DBox(view_widget, bbox3D_corners, color=pg.glColor('b'))

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

def showPC(point_cloud, bbox3D_label=None, prediction_label=None):
    """
    Show point cloud using pyqtopengl,
    bbox3D_label is a ndarray(n, 7) --> (x, y, z, l, w, h, yaw_angle)
    """
    app = QtGui.QApplication([])
    pg.mkQApp()
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    #view_widget = gl.GLViewWidget()
    view_widget = MyGLView()
    view_widget.show()
    view_widget.setWindowTitle('3D Lidar Show Tools')


    ax = gl.GLAxisItem()
    ax.setSize(60, 60, 10)
    view_widget.addItem(ax)

    #b = gl.GLBoxItem()
    #view_widget.addItem(b)


    xgrid = gl.GLGridItem()
    ygrid = gl.GLGridItem()
    zgrid = gl.GLGridItem()
    view_widget.addItem(xgrid)
    view_widget.addItem(ygrid)
    view_widget.addItem(zgrid)

    line = gl.GLLinePlotItem()
    view_widget.addItem(line)

    xgrid.rotate(90, 0, 1, 0)
    ygrid.rotate(90, 1, 0, 0)
    
    x = point_cloud[:, 0]
    y = point_cloud[:, 1]
    z = point_cloud[:, 2]
    selected_area = np.where((y > 0))[0]
    specified_area = np.where((z > -2.0) & (z < 2.5) & (x > 30) & (x < 35))[0]
    #n, bins, patches = plt.hist(point_cloud[selected_area, 3], 10, normed=1, facecolor='green', alpha=0.75)
    #plt.show()
    scatter_plot = gl.GLScatterPlotItem(pos = point_cloud[:, :3], color = pg.glColor('g'), size = 0.5)
    view_widget.addItem(scatter_plot)
    if bbox3D_label is not None:
        #mkPen('y', width = 3, stype=QtCore.Qt.DashLine)
        drawArrow(view_widget, bbox3D_label)
        bbox3D_corners = center_to_corner_box3d(bbox3D_label)
        #draw3DBox(view_widget, bbox3D_corners)
        pts = getBoxPts(bbox3D_corners)
        plt = gl.GLLinePlotItem(pos = pts)
        view_widget.addItem(plt)


    if prediction_label is not None:
        #mkPen('y', width = 3, stype=QtCore.Qt.DashLine)
        bbox3D_corners = center_to_corner_box3d(prediction_label)
        draw3DBox(view_widget, bbox3D_corners, color=pg.glColor('b'))


    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

def center_to_corner_box3d_cam(boxes_center):
    # (N, 7) -> (N, 8, 3)
    N = boxes_center.shape[0]
    ret = np.zeros((N, 8, 3), dtype=np.float32)

    for i in range(N):
        box = boxes_center[i]
        translation = box[0:3]
        [l, w, h] = box[3:6]
        yaw = -box[-1]

        trackletBox = np.array([  # in velodyne coordinates around zero point and without orientation yet
            [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2], \
            [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2], \
            [h / 2, h / 2, h / 2, h / 2, -h / 2, -h / 2, -h / 2, -h / 2]])

        # re-create 3D bounding box in velodyne coordinate system
        rotMat = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0], 
            [0.0, 0.0, 1.0],
            [np.sin(yaw), np.cos(yaw), 0.0]])
        cornerPosInVelo = np.dot(rotMat, trackletBox) + \
            np.tile(translation, (8, 1)).T
        box3d = cornerPosInVelo.transpose()
        ret[i] = box3d


    return ret

def convert_from_kitti_label_to_matrix(kitti_label_file, img_idx):
    """ Input: the file path of kitti label
        Return: the (N, 7) matrix: (x, y, z, l, w, h, theta)
    """
    label_lists = obj_utils.read_labels("/mnt/raid1/data/kitti/training/label_2/", img_idx)
    bbox_label = []
    for i in range(len(label_lists)):
        cur = []
        cur.append(label_lists[i].t[0])
        cur.append(label_lists[i].t[1])
        cur.append(label_lists[i].t[2])
        cur.append(label_lists[i].l)
        cur.append(label_lists[i].w)
        cur.append(label_lists[i].h)
        cur.append(label_lists[i].ry)

        bbox_label.append(cur)
    return np.asarray(bbox_label)

def center_to_corner_box3d(boxes_center):
    # (N, 7) -> (N, 8, 3)
    N = boxes_center.shape[0]
    ret = np.zeros((N, 8, 3), dtype=np.float32)

    for i in range(N):
        box = boxes_center[i]
        translation = box[0:3]
        [l, w, h] = box[3:6]
        yaw = box[-1]

        trackletBox = np.array([  # in velodyne coordinates around zero point and without orientation yet
            [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2], \
            [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2], \
            [h / 2, h / 2, h / 2, h / 2, -h / 2, -h / 2, -h / 2, -h / 2]])

        # re-create 3D bounding box in velodyne coordinate system
        rotMat = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0]])
        cornerPosInVelo = np.dot(rotMat, trackletBox) + \
            np.tile(translation, (8, 1)).T
        box3d = cornerPosInVelo.transpose()
        ret[i] = box3d

    return ret

def drawArrow(w, bbox3D_label):
    for idx in range(bbox3D_label.shape[0]):
        cur_box = bbox3D_label[idx, :]
        
def draw3DBox(w, bbox3D_corners, color=pg.glColor('r')):
    edge_list = [[0, 1], [1, 2], [2, 3], [3, 0], 
                 [4, 5], [5, 6], [6, 7], [7, 4],
                 [0, 4], [1, 5], [2, 6], [3, 7]]
    for box_idx in range(bbox3D_corners.shape[0]):
        cur_box = bbox3D_corners[box_idx, :, :]
        for edge_idx in edge_list:
            pts = cur_box[edge_idx, :]
            plt = gl.GLLinePlotItem(pos = pts, color = color)
            w.addItem(plt)

def getBoxPts(bbox3D_corners):
    edge_list = [[0, 1], [1, 2], [2, 3], [3, 0], 
                 [4, 5], [5, 6], [6, 7], [7, 4],
                 [0, 4], [1, 5], [2, 6], [3, 7]]
    N = bbox3D_corners.shape[0]
    pts = np.zeros((N*12, 2, 3))
    i = 0
    for box_idx in range(bbox3D_corners.shape[0]):
        cur_box = bbox3D_corners[box_idx, :, :]
        for edge_idx in edge_list:
            pts[i] = cur_box[edge_idx, :]
            i += 1

    return pts

def featureMap(feature_map): 
    """
    Show nth level feature map, height map + density map
    """
    f, axarr = plt.subplots(2, 3)
    plt.suptitle('Feature Map')
    for i in range(0, 2):
        for j in range(0, 3):
            ith_height = i*3+j
            if i == 0 and j == 0:
                im = axarr[0, 0].imshow(feature_map[ith_height, :, :])
            else:
                axarr[i, j].imshow(feature_map[ith_height, :, :])
            if ith_height < 5:
                axarr[i, j].set_xlabel('The {}_th height map'.format(i*3+j+1))
            else:
                axarr[i, j].set_xlabel('The density map')

    cbaxes = f.add_axes([0.92, 0.11, 0.02, 0.75])
    cb = plt.colorbar(im, cax = cbaxes)
    plt.show()

def showImgWithPosition(img):
    """
    Show 2D image with their position index
    """
    fig = plt.figure()
    ax = fig.add_subplot(111)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    ax.imshow(img, cmap=cm.jet, interpolation='nearest')
    n_rows, n_cols= img.shape
    def format_coord(x, y):
        col = int(x + 0.5)
        row = int(y + 0.5)
        if col >= 0 and col < n_cols and row >= 0 and row < n_rows:
            z = img[row, col]
            return 'x=%1.4f, y=%1.4f, z=%1.4f'%(x, y, z)
        else:
            return 'x=%1.4f, y=%1.4f'%(x, y)
    ax.format_coord = format_coord
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Utility Test Options')
    parser.add_argument('--use', type=str, nargs='?', default='show_lidar',
                                        help='Show_lidar/Show_featuremap Option')
    parser.add_argument('--pc', type=str, nargs='?', default='/mnt/data_0/kitti/training/velodyne/000000.bin',
                                        help='The path of point cloud')
    parser.add_argument('--pl', type=str, nargs='?', default=None,
                                        help='The detection label path (numpy array)')
    parser.add_argument('--gl', type=str, nargs='?', default=None,
                                        help='The ground truth label path (numpy array)')

    parser.add_argument('--coor', type=str, nargs='?', default='lidar',
                                        help='The default show 3D coordinate system')

    args = parser.parse_args()

    print(args.use)
    if args.use == 'show_lidar':
        pc_name = args.pc
        if pc_name.endswith("bin"):
            img_idx = int(os.path.basename(args.pc)[:-4])
            # By default, if .bin file as point cloud, we show bboxes in camera system rather than lidar system.
            point_cloud = np.fromfile(args.pc, np.float32).reshape(-1, 4)
            frame_calibration_info = calib_utils.read_calibration("/mnt/raid1/data/kitti/training/calib/", 
                                                                  img_idx)
            point_cloud_in_cam = calib_utils.lidar_to_cam_frame(point_cloud[:, :-1], frame_calibration_info) 

        elif pc_name.endswith("npy"):
            # By default, if .npy file as point cloud, we show bboxes in lidar system.
            point_cloud = np.load(args.pc)
        if point_cloud is not None:
            bbox3D_label = None
            prediction_label = None

            # read prediction
            if args.pl is not None:
                prediction_label = np.loadtxt(args.pl)
                if prediction_label is not None:
                    if len(prediction_label.shape) is 1:
                        prediction_label = np.expand_dims(prediction_label, axis=0)
                    if args.coor == "lidar":
                        prediction_label = prediction_label[:, 1:]
                    elif args.coor == "camera":
                        prediction_label = prediction_label[:, :7]
                    print("---prediction_label is {}".format(prediction_label))

            # read groundtruth 
            if args.gl is not None:
                if args.coor == "lidar":
                    bbox3D_label = np.loadtxt(args.gl)
                elif args.coor == "camera":
                    bbox3D_label = convert_from_kitti_label_to_matrix(args.pl, img_idx)

            if args.coor == "lidar":        
                showPC(point_cloud, 
                    bbox3D_label=bbox3D_label, 
                    prediction_label=prediction_label)
            elif args.coor == 'camera':
                showPCInCam(point_cloud_in_cam, 
                    bbox3D_label=bbox3D_label, 
                    prediction_label=prediction_label)
        
        else:
            print("-------------Loading pc is wrong ----------------------")



    elif args.use == 'show_featuremap':
        # load feature map, currently show 6 layers
        pc_name = args.pc
        if pc_name.endswith(".npy"):
            feature_map = np.load(pc_name)
            pdb.set_trace()
            featureMap(feature_map[0, :, :, :].transpose(2, 0, 1))

#        bin_name = "/mnt/raid1/Research/lidar-processing/data.bin"
#        bin_data = np.fromfile(bin_name, dtype = np.uint8).reshape(6, 600, 600)
