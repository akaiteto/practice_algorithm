import numpy as np
import  cv2
import open3d as o3d
import utils_pcl as util_3d
import copy

class PoseEstimation():
    color_intrinsics = None
    depth_intrinsics = None
    depth_scale = None
    fx = None
    fy = None
    ppx = None
    ppy = None

    class BaseOpenCV():
        def estimate_pose(self,kpts0, kpts1, dic=None, conf=0.99999):
            if len(kpts0) < 5:
                return None
            fx = self.fx
            fy = self.fy
            ppx = self.ppx
            ppy = self.ppy
            K0 = K1 = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])

            thresh = 1
            f_mean = np.mean([K0[0, 0], K1[1, 1], K0[0, 0], K1[1, 1]])
            norm_thresh = thresh / f_mean

            kpts0 = (kpts0 - K0[[0, 1], [2, 2]][None]) / K0[[0, 1], [0, 1]][None]
            kpts1 = (kpts1 - K1[[0, 1], [2, 2]][None]) / K1[[0, 1], [0, 1]][None]

            E, mask = cv2.findEssentialMat(
                kpts0, kpts1, np.eye(3), threshold=norm_thresh, prob=conf,
                method=cv2.RANSAC)

            assert E is not None

            best_num_inliers = 0
            ret = None
            for _E in np.split(E, len(E) / 3):
                n, R, t, _ = cv2.recoverPose(
                    _E, kpts0, kpts1, np.eye(3), 1e9, mask=mask)
                if n > best_num_inliers:
                    best_num_inliers = n
                    ret = (R, t[:, 0], mask.ravel() > 0)

            return ret

    class BaseCPD():
        from probreg import cpd
        def CPD_rejister(self,source, target, w=0.0, maxiter=50, tol=0.001):
            from probreg import cpd
            import copy
            type = 'rigid'
            tf_param, _, _ = cpd.registration_cpd(source, target, type, w, maxiter, tol)

            result = copy.deepcopy(source)
            result.points = tf_param.transform(result.points)

            return result, tf_param.rot, tf_param.t, tf_param.scale


        def estimate_pose(self,kpts0, kpts1, dic):
            if kpts0.shape != kpts1.shape: assert False, "マッチ件数不一致"

            fx = self.fx
            fy = self.fy
            ppx = self.ppx
            ppy = self.ppy
            depth_scale = self.depth_scale

            depth0 = dic["depth0"]
            depth1 = dic["depth1"]
            rgb0 = dic["color0"]
            rgb1 = dic["color1"]
            output_feature_ply0 = dic["output_feature_ply0"]
            output_feature_ply1 = dic["output_feature_ply1"]

            output_depth0 = np.zeros(depth0.shape)
            output_depth1 = np.zeros(depth1.shape)

            for idx in range(len(kpts0)):
                w0 = int(kpts0[idx][0])
                h0 = int(kpts0[idx][1])
                w1 = int(kpts1[idx][0])
                h1 = int(kpts1[idx][1])

                output_depth0[h0][w0] = depth0[h0][w0]
                output_depth1[h1][w1] = depth0[h1][w1]

            util_3d.generate_pointcloud(fW=fx, fH=fy, centerW=ppx, centerH=ppy,scalingFactor=depth_scale,
                                        rgb=rgb0, depth=output_depth0, ply_file=output_feature_ply0)
            util_3d.generate_pointcloud(fW=fx, fH=fy, centerW=ppx, centerH=ppy,scalingFactor=depth_scale,
                                        rgb=rgb1, depth=output_depth1, ply_file=output_feature_ply1)

            pcl_feat0 = util_3d.read_point_cloud(output_feature_ply0)
            pcl_feat1 = util_3d.read_point_cloud(output_feature_ply1)

            w = 0.0
            maxiter = 1000
            tol = 0.001
            result, rot, t, scale = self.CPD_rejister(pcl_feat1, pcl_feat0,w,maxiter,tol)

            lastRow = np.array([[0, 0, 0, 1]])
            R = np.array(rot)
            t = np.array([t]).T.reshape(3)
            R = scale * R

            return [R,t,None]
