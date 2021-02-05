import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import match_superglue as mt
import pose_estimate as ps
import utils_pcl as util_3d
import utils_image as util_2d
import eval_registration as eval

class bag_ctl():
    pathCSV=None
    pathImg=None
    pathPLY=None
    pathWORK=None

    def __init__(self, pathCSV, pathImg,pathPLY,pathWORK):
        self.pathCSV = pathCSV
        self.pathImg = pathImg
        self.pathPLY = pathPLY
        self.pathWORK = pathWORK

    def setAlgorithm(self,param):
        p_match = param["match"]
        p_pose = param["est_pose"]
        p_eval = param["eval"]

        # マッチング
        if p_match == "glue":
            self.match = mt.match_glue()
        elif p_match == "o3d_graph":
            self.pose = mt.match_o3d_posegraph()
        else:
            assert False,"マッチングアルゴリズムが未選択"

        # カメラ姿勢推定
        if p_pose == "opencv":
            self.pose = ps.PoseEstimation.BaseOpenCV()
        elif p_pose == "cpd":
            self.pose = ps.PoseEstimation.BaseCPD()
        else:
            assert False,"カメラ姿勢推定アルゴリズムが未選択"

        # 位置合わせ評価方法
        if p_eval == "Reproject_3Dto2D":
            self.eval = eval.Evaluate.Reproject_3Dto2D()
        else:
            assert False,"位置合わせ評価方法が未選択"

    def get_filter(self):
        align_to = rs.stream.color
        align = rs.align(align_to)
        dic = {"align": align}
        return dic

    def get_devinfo(self,pipeline):
        profile = pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        dic = {"depth_scale": depth_scale, "color_intrinsics": color_intrinsics, "depth_intrinsics": depth_intrinsics}
        return dic

    def get_dicMatch(self,depth0,depth1,color0,color1):
        dic_match = {}
        dic_match["depth0"] = depth0
        dic_match["depth1"] = depth1
        dic_match["color0"] = color0
        dic_match["color1"] = color1


        return dic_match

    def get_dicPose(self,depth0,depth1,color0,color1):
        dic_pose = {}
        dic_pose["depth0"] = depth0
        dic_pose["depth1"] = depth1
        dic_pose["color0"] = color0
        dic_pose["color1"] = color1

        dic_pose["output_feature_ply0"] = self.pathWORK + "\\feature0.ply"
        dic_pose["output_feature_ply1"] = self.pathWORK + "\\feature1.ply"

        return dic_pose

    def extractFrameFromBag(self,bagfilepath):
        #アルゴリズムセット
        dic = {"match":"o3d_graph","est_pose":"cpd","eval":"Reproject_3Dto2D"}
        self.setAlgorithm(dic)

        #realsense準備
        config = rs.config()
        rs.config.enable_device_from_file(config, bagfilepath)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
        try:
            pipeline = rs.pipeline()
            profile = pipeline.start(config)
        except:
            assert False, "error:fail to load bag file setting.The settings in config.enable_stream may be incorrect"
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)

        # デバイス情報取得
        dic_dev = self.get_devinfo(pipeline)

        # デバイス情報セット
        self.pose.color_intrinsics = dic_dev["color_intrinsics"]
        self.pose.depth_intrinsics = dic_dev["depth_intrinsics"]
        self.pose.depth_scale = dic_dev["depth_scale"]
        self.pose.fx = dic_dev["color_intrinsics"].fx
        self.pose.fy = dic_dev["color_intrinsics"].fy
        self.pose.ppx = dic_dev["color_intrinsics"].ppx
        self.pose.ppy = dic_dev["color_intrinsics"].ppy
        self.K =  np.array([[self.pose.fx, 0, self.pose.ppx], [0, self.pose.fy, self.pose.ppy], [0, 0, 1]])

        # フィルターインスタンス取得
        dic_filter = self.get_filter()
        align = dic_filter["align"]

        #フレーム取得
        fpscnt = 0
        frameNum = 0
        bfr_image = None
        cur_image = None
        try:
            while True:
                try:
                    frames = pipeline.wait_for_frames()
                except:
                    break

                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not depth_frame or not color_frame:
                    continue
                color_numpy = np.asanyarray(color_frame.get_data())
                depth_numpy = np.asanyarray(depth_frame.get_data())
                if np.sum(depth_numpy)==0:
                    continue

                frameNum += 1
                fpscnt += 1
                if fpscnt >= 20:
                    cur_frame = frameNum
                    cur_image = color_numpy
                    cur_depth = depth_numpy
                    fpscnt = 0
                    cur_desc = "\\" + str(cur_frame)

                    #フレーム保存
                    np.savetxt(self.pathCSV + cur_desc + ".csv", depth_numpy, delimiter=',', fmt='%10f')
                    cv2.imwrite(self.pathImg + cur_desc + ".png", cv2.cvtColor(color_numpy, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_PNG_COMPRESSION, 0])

                    if bfr_image is not None:
                        color0 = bfr_image
                        depth0 = bfr_depth
                        desc0 = bfr_desc
                        color1 = cur_image
                        depth1 = cur_depth
                        desc1 = cur_desc

                        #マッチング
                        dicMatch = self.get_dicMatch(depth0 = depth0,depth1 = depth1,color0 = color0,color1 = color1)
                        kpts0, kpts1 = self.match.extract_matching(color0, color1,dicMatch)
                        #カメラ姿勢推定
                        dicPose = self.get_dicPose(depth0 = depth0,depth1 = depth1,color0 = color0,color1 = color1)
                        ret = self.pose.estimate_pose(kpts0, kpts1,dicPose)

                        if ret is None:
                            err_t, err_R = np.inf, np.inf
                            print("カメラ姿勢推定失敗",err_t, err_R)
                            continue
                        else:
                            R, t, inliers = ret
                            # 点群作成
                            util_3d.generate_pointcloud(fW=self.pose.fx, fH=self.pose.fy, centerW=self.pose.ppx,
                                                        centerH=self.pose.ppy,
                                                        scalingFactor=self.pose.depth_scale,
                                                        rgb=color0, depth=depth0,
                                                        ply_file=self.pathPLY + desc0 + ".ply")

                            util_3d.generate_pointcloud(fW=self.pose.fx, fH=self.pose.fy, centerW=self.pose.ppx,
                                                        centerH=self.pose.ppy,
                                                        scalingFactor=self.pose.depth_scale,
                                                        rgb=color1, depth=depth1,
                                                        ply_file=self.pathPLY + desc1 + ".ply")

                            # 位置合わせ実行
                            pcl1 = util_3d.read_point_cloud(self.pathPLY + desc1 + ".ply")
                            P = util_3d.ConvRT2TransXYZW(R,t)
                            pcl1_trans = util_3d.ApplyPmat4x4(pcl1,P)
                            util_3d.write_point_cloud(self.pathPLY + cur_desc + "_trans.ply",pcl1_trans)

                            # 位置合わせ評価
                            val_eval = self.eval.Evaluate(pcl1,color0,self.K,flgSave=False)
                            print("\t",val_eval)

                            # bfr_frame = frameNum
                            # bfr_depth = cur_depth
                            # bfr_image = cur_image

                    else:
                        bfr_frame = frameNum
                        bfr_depth = cur_depth
                        bfr_image = cur_image
                        bfr_desc = "\\" + str(bfr_frame)



            print("finish")
        finally:
            pipeline.stop()


if __name__ == '__main__':
    bag_ctl = bag_ctl(pathCSV = r"D:\test\aaaaaa\depth",
                        pathImg = r"D:\test\aaaaaa\img",
                        pathPLY = r"D:\test\aaaaaa\input",
                        pathWORK = r"D:\test\aaaaaa\tmp")
    bag_ctl.extractFrameFromBag(bagfilepath=r"C:\Users\dobashi\Documents\20210205_145804.bag")