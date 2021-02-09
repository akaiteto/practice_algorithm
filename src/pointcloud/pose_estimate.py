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
    class Common():
        # https://github.com/tronphan/3D-Rendering/blob/51f5fbdd1d88b8fd1befd631b8f8f2cdd23822cc/Reconstruction/opencv_pose_estimation.py
        def estimate_3D_transform(self,input_xyz_s, input_xyz_t):
            # compute H
            xyz_s = copy.copy(input_xyz_s)
            xyz_t = copy.copy(input_xyz_t)
            n_points = xyz_s.shape[1]
            mean_s = np.mean(xyz_s, axis=1)
            mean_t = np.mean(xyz_t, axis=1)
            mean_s.shape = (3, 1)
            mean_t.shape = (3, 1)
            xyz_diff_s = xyz_s - np.tile(mean_s, [1, n_points])
            xyz_diff_t = xyz_t - np.tile(mean_t, [1, n_points])
            H = np.matmul(xyz_diff_s, xyz_diff_t.transpose())
            # solve system
            U, s, V = np.linalg.svd(H)
            R_approx = np.matmul(V.transpose(), U.transpose())
            if np.linalg.det(R_approx) < 0.0:
                det = np.linalg.det(np.matmul(U, V))
                D = np.identity(3)
                D[2, 2] = det
                R_approx = np.matmul(U, np.matmul(D, V))
            t_approx = mean_t - np.matmul(R_approx, mean_s)
            return R_approx, t_approx

        # https://github.com/tronphan/3D-Rendering/blob/51f5fbdd1d88b8fd1befd631b8f8f2cdd23822cc/Reconstruction/opencv_pose_estimation.py
        def estimate_3D_transform_RANSAC(self,pts_xyz_s, pts_xyz_t):
            max_iter = 1000
            max_distance = 0.05
            n_sample = 5
            n_points = pts_xyz_s.shape[1]
            Transform_good = np.identity(4)
            max_inlier = n_sample
            inlier_vec_good = []
            success = False

            if n_points < n_sample:
                return False, np.identity(4), []

            for i in range(max_iter):

                # sampling
                rand_idx = np.random.randint(n_points, size=n_sample)
                sample_xyz_s = pts_xyz_s[:, rand_idx]
                sample_xyz_t = pts_xyz_t[:, rand_idx]
                R_approx, t_approx = self.estimate_3D_transform(sample_xyz_s, sample_xyz_t)

                # evaluation
                diff_mat = pts_xyz_t - (np.matmul(R_approx, pts_xyz_s) +
                                        np.tile(t_approx, [1, n_points]))
                diff = [np.linalg.norm(diff_mat[:, i]) for i in range(n_points)]
                n_inlier = len([1 for diff_iter in diff if diff_iter < max_distance])

                # note: diag(R_approx) > 0 prevents ankward transformation between
                # RGBD pair of relatively small amount of baseline.
                if (n_inlier > max_inlier) and (np.linalg.det(R_approx) != 0.0) and \
                        (R_approx[0, 0] > 0 and R_approx[1, 1] > 0 and R_approx[2, 2] > 0):
                    Transform_good[:3, :3] = R_approx
                    Transform_good[:3, 3] = [t_approx[0], t_approx[1], t_approx[2]]
                    max_inlier = n_inlier
                    inlier_vec = [id_iter for diff_iter, id_iter \
                                  in zip(diff, range(n_points)) \
                                  if diff_iter < max_distance]
                    inlier_vec_good = inlier_vec
                    success = True

            return success, Transform_good, inlier_vec_good


        def refinePose(self,pcl_target, P_target,img_main,K):
            f = K[0][0]
            ppw = K[0][2]
            pph = K[1][2]

            pts_Main = []
            pts_Target = []

            target_height = img_main.shape[0]
            target_width = img_main.shape[1]
            cnt=0
            for idx in range(len(pcl_target.points)):
                pt = np.array(pcl_target.points[idx])
                pt = np.array([pt[0],pt[1],pt[2],1]).reshape(4,1)
                w_Main, h_Main = util_3d.TransformPointC2I(pt, K)
                if target_height < h_Main or 0 > h_Main:
                    continue
                if target_width < w_Main or 0 > w_Main:
                    continue

                pt_trans=np.dot(P_target,pt)
                w_Target, h_Target = util_3d.TransformPointC2I(pt_trans, K)
                if target_height < h_Target or 0 > h_Target:
                    continue
                if target_width < w_Target or 0 > w_Target:
                    continue
                cnt+=1
                pts_Main.append([w_Main,h_Main])
                pts_Target.append([w_Target,h_Target])

            pts_Main=np.array(pts_Main)
            pts_Target=np.array(pts_Target)
            pts_Main = np.int32(pts_Main + 0.5)
            pts_Target = np.int32(pts_Target + 0.5)

            [E, mask] = cv2.findEssentialMat(pts_Main,
                                             pts_Target,
                                             focal=f,
                                             pp=(ppw, pph),
                                             method=cv2.RANSAC,
                                             prob=0.999,
                                             threshold=1.0)

            if mask is None:
                print("false")

            cnt_success = np.count_nonzero(mask != 0)
            print(cnt_success/len(mask))
            return cnt_success/len(mask)


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
