import random
import csv
import numpy as np
import  cv2
import match_superglue as mt
import open3d as o3d
import utils_pcl as util_3d
import utils_image as util_2d
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

        def refinePose(self,pcl_target,P_target,depthM,depthT,img_main,img_target,K):
            return self.refinePose_3Dbased_Ransac(pcl_target, P_target, depthM, depthT, img_main, img_target, K)

        def refinePose_3Dbased_Ransac(self,pcl_target,P_target,depthM,depthT,img_main,img_target,K):
            '''
            推定したカメラ姿勢より、マッチングするポイント抽出->ransacで推定
            1.位置合わせ前のターゲット点群の3Dポイント取得

            2.位置合わせ後のターゲット点群の3Dポイント取得
            3.処理後のポイントを2Dポイント化

            '''
            f = K[0][0]
            ppw = K[0][2]
            pph = K[1][2]

            pts2D_Main = []
            pts2D_Target = []
            pts3D_Main = []
            pts3D_Target = []

            target_height = img_main.shape[0]
            target_width = img_main.shape[1]
            cnt=0

            num_extract_points=50
            lst_sampling = random.sample(list(range(len(pcl_target.points))), num_extract_points)

            # for idx in range(len(pcl_target.points)):
            for idx in lst_sampling:
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

                dM = int(depthM[int(h_Main)][int(w_Main)])
                dT = int(depthT[int(h_Target)][int(w_Target)])

                if dM == 0 or dT == 0: continue

                XYZ_Main = util_3d.TransformPointI2C([w_Main,h_Main,dM],K)
                XYZ_Target = util_3d.TransformPointI2C([w_Target,h_Target,dT],K)

                pts2D_Main.append([w_Main,h_Main])
                pts2D_Target.append([w_Target,h_Target])
                pts3D_Main.append(XYZ_Main)
                pts3D_Target.append(XYZ_Target)
                cnt+=1

            # マッチング画像出力
            img_match = util_2d.getMatchImage(srcImg=img_main,dstImg=img_target,srcPts=pts2D_Main,dstPts=pts2D_Target)
            cv2.imwrite("match.png", cv2.cvtColor(img_match, cv2.COLOR_BGR2RGB),[cv2.IMWRITE_PNG_COMPRESSION, 0])

            # マッチング点群出力
            pclM = util_3d.generate_pointcloud_fromXYZpoints(pts3D_Main)
            util_3d.write_point_cloud("pclM.ply",pclM)
            pclT = util_3d.generate_pointcloud_fromXYZpoints(pts3D_Target)
            util_3d.write_point_cloud("pclT.ply",pclT)

            nplst = np.array(pts3D_Main).shape
            pts3D_Main = np.array(pts3D_Main).reshape(nplst[0], nplst[1])
            pts3D_Target = np.array(pts3D_Target).reshape(nplst[0], nplst[1])

            pts_xyz_Main = np.zeros([3, pts3D_Main.shape[0]])
            pts_xyz_Target = np.zeros([3, pts3D_Main.shape[0]])

            cnt = 0
            for i in range(pts3D_Main.shape[0]):
                xyz_M = pts3D_Main[i]
                pts_xyz_Main[:, cnt] = xyz_M
                xyz_T = pts3D_Target[i]
                pts_xyz_Target[:, cnt] = xyz_T
                cnt = cnt + 1

            pts_xyz_Main = pts_xyz_Main[:, :cnt]
            pts_xyz_Target = pts_xyz_Target[:, :cnt]

            success, trans, inlier_id_vec =self.estimate_3D_transform_RANSAC(pts_xyz_Main, pts_xyz_Target)
            print(success, trans, inlier_id_vec)

        def integrate(self,depthM,depthT,img_main,img_target,K):
            return self.integrate_2Dbased_Ransac(depthM,depthT,img_main,img_target,K)

        def integrate_2Dbased_Ransac(self,depthM,depthT,img_main,img_target,K):
            '''
            2Dイメージマッチング->3D化->pose推定
            '''
            f = K[0][0]
            ppw = K[0][2]
            pph = K[1][2]

            # 2Dベースの位置合わせ
            # Match = mt.match_opencv()
            Match = mt.match_glue()
            pts2D_Main, pts2D_Target = Match.extract_matching(img_main,img_target)

            # 剛体変換取得
            # Essential matrix is made for masking inliers
            pts_s_int = np.int32(pts2D_Main + 0.5)
            pts_t_int = np.int32(pts2D_Target + 0.5)
            [E, mask] = cv2.findEssentialMat(pts_s_int,
                                             pts_t_int,
                                             focal=f,
                                             pp=(ppw, pph),
                                             method=cv2.RANSAC,
                                             prob=0.999,
                                             threshold=1.0)
            if mask is None:
                return False, None, None

            # マッチング画像出力
            img_match = util_2d.getMatchImage(srcImg=img_main,dstImg=img_target,srcPts=pts2D_Main,dstPts=pts2D_Target)
            cv2.imwrite("match.png", cv2.cvtColor(img_match, cv2.COLOR_BGR2RGB),[cv2.IMWRITE_PNG_COMPRESSION, 0])

            pts3D_Main = []
            pts3D_Target = []
            for i in range(pts2D_Main.shape[0]):
                if mask[i]:
                    w_Main, h_Main = pts2D_Main[i][0],pts2D_Main[i][1]
                    dM = int(depthM[int(h_Main)][int(w_Main)])

                    w_Target, h_Target = pts2D_Target[i][0],pts2D_Target[i][1]
                    dT = int(depthT[int(h_Target)][int(w_Target)])
                    if dM==0 or dT==0:continue

                    XYZ_Main = util_3d.TransformPointI2C([w_Main,h_Main,dM],K)
                    XYZ_Target = util_3d.TransformPointI2C([w_Target,h_Target,dT],K)

                    if not np.all(XYZ_Main ==XYZ_Target):
                        pts3D_Main.append(XYZ_Main)
                        pts3D_Target.append(XYZ_Target)


            # マッチング点群出力
            pclM = util_3d.generate_pointcloud_fromXYZpoints(pts3D_Main)
            util_3d.write_point_cloud("pclM.ply",pclM)
            pclT = util_3d.generate_pointcloud_fromXYZpoints(pts3D_Target)
            util_3d.write_point_cloud("pclT.ply",pclT)

            nplst = np.array(pts3D_Main).shape
            pts3D_Main = np.array(pts3D_Main).reshape(nplst[0], nplst[1])
            pts3D_Target = np.array(pts3D_Target).reshape(nplst[0], nplst[1])

            pts_xyz_Main = np.zeros([3, pts3D_Main.shape[0]])
            pts_xyz_Target = np.zeros([3, pts3D_Main.shape[0]])

            cnt = 0
            for i in range(pts3D_Main.shape[0]):
                if mask[i]:
                    xyz_M = pts3D_Main[i]
                    pts_xyz_Main[:, cnt] = xyz_M
                    xyz_T = pts3D_Target[i]
                    pts_xyz_Target[:, cnt] = xyz_T
                    cnt = cnt + 1

            pts_xyz_Main = pts_xyz_Main[:, :cnt]
            pts_xyz_Target = pts_xyz_Target[:, :cnt]

            cpd = PoseEstimation.BaseCPD()
            success, trans, inlier_id_vec = cpd.estimate_pose_RANSAC(pts_xyz_Main,pts_xyz_Target)

            return success, trans, inlier_id_vec

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

        def estimate_pose_RANSAC(self,pts_xyz_s, pts_xyz_t):
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

                # マッチング点群出力
                sample_xyz_s = list(sample_xyz_s.T)
                sample_xyz_t = list(sample_xyz_t.T)
                pcl_s = util_3d.generate_pointcloud_fromXYZpoints(sample_xyz_s)
                pcl_t = util_3d.generate_pointcloud_fromXYZpoints(sample_xyz_t)

                w = 0.0
                maxiter = 1000
                tol = 0.001
                result, R_approx, t_approx, scale_approx = self.CPD_rejister(pcl_s, pcl_t, w, maxiter, tol)
                t_approx = t_approx.reshape(3,1)

                P = util_3d.ConvRT2TransXYZW(scale_approx*R_approx, t_approx.reshape(3))


                # マッチング点群出力
                pcl_s_all = util_3d.generate_pointcloud_fromXYZpoints(list(pts_xyz_s.T))
                pcl_t_all = util_3d.generate_pointcloud_fromXYZpoints(list(pts_xyz_t.T))

                pcl_s_all_trans = util_3d.ApplyPmat4x4(pcl_s_all, P)

                diff = pcl_s_all_trans.compute_point_cloud_distance(pcl_t_all)
                diff = np.array(diff)*0.00025

                n_inlier = len([1 for diff_iter in diff if diff_iter < max_distance])
                if (n_inlier > max_inlier) and (np.linalg.det(R_approx) != 0.0) and \
                        (R_approx[0, 0] > 0 and R_approx[1, 1] > 0 and R_approx[2, 2] > 0):
                    Transform_good=P

                    util_3d.write_point_cloud("pclM.ply", pcl_s_all_trans)
                    util_3d.write_point_cloud("pclT.ply", pcl_t_all)

                    max_inlier = n_inlier
                    inlier_vec = [id_iter for diff_iter, id_iter \
                                  in zip(diff, range(n_points)) \
                                  if diff_iter < max_distance]
                    inlier_vec_good = inlier_vec
                    success = True

            print(n_inlier,max_inlier,pts_xyz_s.shape)
            return success, Transform_good, inlier_vec_good

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
