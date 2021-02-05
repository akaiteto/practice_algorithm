

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

def Reprojection_3Dto2D(pcl_target,img_main,K):
    imgM_T2M = np.zeros(img_main.shape, dtype=np.uint8)
    imgM_hole_on_T = np.zeros(img_main.shape, dtype=np.uint8)

    UVheight = img_main.shape[0]
    UVwidth = img_main.shape[1]
    for idx in range(len(pcl_target.points)):
        XYZ = pcl_target.points[idx]
        RGB = pcl_target.colors[idx]
        w,h = TransformPointC2I(XYZ,K)
        uv = [int(h),int(w)]

        if UVheight < h or 0 > h:
            continue

        if UVwidth < w  or 0 > w:
            continue

        imgM_T2M[uv[0]][uv[1]][0] = RGB[0]*255
        imgM_T2M[uv[0]][uv[1]][1] = RGB[1]*255
        imgM_T2M[uv[0]][uv[1]][2] = RGB[2]*255

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

def generate_pointcloud(fW,fH,centerW,centerH,scalingFactor,rgb, depth, ply_file):
    if rgb.shape[0] != depth.shape[0] or rgb.shape[1] != depth.shape[1]:
        raise Exception("Color and depth image do not have the same resolution.")
    points = []
    for w in range(rgb.shape[1]):
        for h in range(rgb.shape[0]):
            color = rgb[h][w]
            Z = depth[h][w] / scalingFactor
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


