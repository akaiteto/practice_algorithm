

from PIL import Image
import numpy as np
import open3d as o3d
import copy
import cv2

def read_rgbd(color, depth, max_depth=3.0):
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_trunc=max_depth,
        convert_rgb_to_intensity=False)
    return rgbd_image

def TransformPointC2I(XYZ_C, K):
    d = XYZ_C[2]
    w = float(float(XYZ_C[0] * K[0][0]) / d) + K[0][2]
    h = float(float(XYZ_C[1] * K[1][1]) / d) + K[1][2]
    return w, h

def TransformPointI2C(pixel2D, K):
    depthZ = pixel2D[2]
    X = float(float(pixel2D[0] - K[0][2]) * depthZ / K[0][0])
    Y = float(float(pixel2D[1] - K[1][2]) * depthZ / K[1][1])
    Z = depthZ
    CameraPos3D = np.array([[X], [Y], [Z]])
    return CameraPos3D

def ConvRT2TransXYZW(R, T):
    trans = np.array([[R[0][0], R[0][1], R[0][2], T[0][0]],
                      [R[1][0], R[1][1], R[1][2], T[1][0]],
                      [R[2][0], R[2][1], R[2][2], T[2][0]],
                      [0, 0, 0, 1]])
    return trans

def ConvP2RT(trans):
    tarR = np.array([[trans[0][0], trans[0][1], trans[0][2]],
                     [trans[1][0], trans[1][1], trans[1][2]],
                     [trans[2][0], trans[2][1], trans[2][2]]])

    tart = np.array([[trans[0][3]], [trans[1][3]], [trans[2][3]]])
    return tarR, tart

def getInverseRt(R, t):
    invR = np.linalg.inv(R)
    invT = (-1) * np.dot(invR, t)

    return invR,invT

def Reprojection_3Dto2D(pcl_target,img_main,K,imgM_T2M=None):
    if imgM_T2M is None:
        imgM_T2M = np.zeros(img_main.shape, dtype=np.uint8)
    imgM_hole_on_T = np.zeros(img_main.shape, dtype=np.uint8)

    chkarray =np.array([0,0,0])
    target_height = imgM_T2M.shape[0]
    target_width = imgM_T2M.shape[1]
    origin_height = img_main.shape[0]
    origin_width = img_main.shape[1]

    for idx in range(len(pcl_target.points)):
        XYZ = pcl_target.points[idx]
        RGB = pcl_target.colors[idx]
        w,h = TransformPointC2I(XYZ,K)
        uv = [int(h),int(w)]

        if target_height < h or 0 > h:
            continue

        if target_width < w  or 0 > w:
            continue

        if np.array_equal(chkarray, imgM_T2M[uv[0]][uv[1]]):
            imgM_T2M[uv[0]][uv[1]][0] = RGB[0]*255
            imgM_T2M[uv[0]][uv[1]][1] = RGB[1]*255
            imgM_T2M[uv[0]][uv[1]][2] = RGB[2]*255

        if (origin_height < h or 0 > h) and (origin_width < w  or 0 > w):
            imgM_hole_on_T[uv[0]][uv[1]][0] = img_main[uv[0]][uv[1]][0]
            imgM_hole_on_T[uv[0]][uv[1]][1] = img_main[uv[0]][uv[1]][1]
            imgM_hole_on_T[uv[0]][uv[1]][2] = img_main[uv[0]][uv[1]][2]

    return imgM_T2M,imgM_hole_on_T

def ConvRT2TransXYZW(R, T):
    trans = np.array([[R[0][0], R[0][1], R[0][2], T[0]],
                      [R[1][0], R[1][1], R[1][2], T[1]],
                      [R[2][0], R[2][1], R[2][2], T[2]],
                      [0, 0, 0, 1]])
    return trans

def read_point_cloud(path):
    pcl = o3d.io.read_point_cloud(path,print_progress=False)
    if len(pcl.points) == 0: assert False, "error : fail to load pointcloud  = " + path
    return pcl

def write_point_cloud(path,pcl):
    assert o3d.io.write_point_cloud(path, pcl,print_progress=False),"error : fail to write pointcloud  = " + path

def ApplyPmat4x4(pcl,P):
    pcl_tmp =copy.copy(pcl)
    return pcl_tmp.transform(P)

def generate_pointcloud_fromXYZpoints(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd

def generate_pointcloud(fW,fH,centerW,centerH,rgb, depth, ply_file,scalingFactor=1.0):
    if rgb.shape[0] != depth.shape[0] or rgb.shape[1] != depth.shape[1]:
        raise Exception("Color and depth image do not have the same resolution.")
    points = []
    for w in range(rgb.shape[1]):
        for h in range(rgb.shape[0]):
            color = rgb[h][w]
            if scalingFactor!=1.0:
                Z = depth[h][w] / scalingFactor
            else:
                Z = depth[h][w]
            if Z == 0: continue
            X = (w - centerW) * Z / fW
            Y = (h - centerH) * Z / fH
            points.append("%f %f %f %d %d %d 0\n" % (X, Y, Z, color[0], color[1], color[2]))

    file = open(ply_file, "w")
    file.write('''ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
end_header
%s
''' % (len(points), "".join(points)))
    file.close()


if __name__ == '__main__':
    pass
    # rgb_file = "TUM_FMT/rgb/1603868952.458810.png"
    # depth_file = "TUM_FMT/depth/1603868952.465590.png"
    # ply_file = "TUM_FMT/test.ply"
    # generate_pointcloud(rgb_file, depth_file, ply_file)


