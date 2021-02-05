import cv2
from otherlib.SuperGluePretrainedNetwork.models.matching import Matching
from otherlib.SuperGluePretrainedNetwork.models.utils import (frame2tensor)

import open3d as o3d
import numpy as np
import torch
import utils_pcl as util_3d

torch.set_grad_enabled(False)
class match_glue:
    def __init__(self):
        self.device = "cpu"
        self.config = {'superpoint': {'nms_radius': 4, 'keypoint_threshold': 0.005, 'max_keypoints': 1024},
                  'superglue': {'weights': 'indoor', 'sinkhorn_iterations': 20, 'match_threshold': 0.2}}
        self.matching = Matching(self.config).eval().to("cpu")

    def extract_matching(self,image0, image1):
        image0 = cv2.cvtColor(image0, cv2.COLOR_BGR2GRAY)
        image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)

        image0, inp0 = self.read_image(image0, self.device)
        image1, inp1 = self.read_image(image1, self.device)

        pred = self.matching({'image0': inp0, 'image1': inp1})
        pred = {k: v[0].cpu().numpy() for k, v in pred.items()}
        kpts0, kpts1 = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        # Keep the matching keypoints.
        valid = matches > -1
        mkpts0 = kpts0[valid]
        mkpts1 = kpts1[matches[valid]]
        mconf = conf[valid]

        return mkpts0, mkpts1

    def read_image(self,image, device):
        w, h = image.shape[1], image.shape[0]
        inp = frame2tensor(image, device)
        return image, inp

class match_o3d_posegraph:

    def __init__(self):
        self.pose_graph = o3d.pipelines.registration.PoseGraph()
        self.trans_odometry = np.identity(4)
        self.pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(self.trans_odometry))

    def extract_matching(self,image0, image1,dicMatch):
        depth0 = dicMatch["depth0"]
        depth1 = dicMatch["depth1"]
        color0 = dicMatch["color0"]
        color1 = dicMatch["color1"]

        rgbd0 = util_3d.read_rgbd(color=color0,depth=depth0)
        rgbd1 = util_3d.read_rgbd(color=color1,depth=depth1)

        exit()
        return mkpts0, mkpts1


if __name__ == '__main__':
    input1 = r"D:\test\git\practice_algorithm\src\otherlib\SuperGluePretrainedNetwork\assets\scannet_sample_images\scene0726_00_frame-000135.jpg"
    input2 = r"D:\test\git\practice_algorithm\src\otherlib\SuperGluePretrainedNetwork\assets\scannet_sample_images\scene0726_00_frame-000210.jpg"


    device = "cpu"
    rot0 = 0
    image0, inp0 = read_image(input1, device)
    image1, inp1 = read_image(input2, device)