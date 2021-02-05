import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import match_superglue as mt
import pose_estimate as ps

class bag_ctl():
    def setAlgorithm(self,param):
        p_match = param["match"]
        p_pose = param["est_pose"]
        # マッチング
        if p_match == "glue":
            self.match = mt.match_glue()
        else:
            assert False,"カメラ姿勢推定アルゴリズムが未選択"

        # カメラ姿勢推定
        if p_pose == "opencv":
            self.pose = ps.BaseOpenCV()
        else:
            assert False,"カメラ姿勢推定アルゴリズムが未選択"


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

    def extractFrameFromBag(self,bagfilepath, pathCSV, pathImg):
        #アルゴリズムセット
        dic = {"match":"glue","est_pose":"opencv"}
        self.setAlgorithm(dic)

        #realsense準備
        config = rs.config()
        rs.config.enable_device_from_file(config, bagfilepath)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        try:
            pipeline = rs.pipeline()
            profile = pipeline.start(config)
        except:
            assert False, "error:fail to load bag file setting.The settings in config.enable_stream may be incorrect"

        playback = profile.get_device().as_playback()
        playback.set_real_time(False)

        # デバイス情報取得
        dic_dev = self.get_devinfo(pipeline)
        color_intrinsics = dic_dev["color_intrinsics"]
        fx = color_intrinsics.fx
        fy = color_intrinsics.fy
        ppx = color_intrinsics.ppx
        ppy = color_intrinsics.ppy
        K =np.array([[fx,0,ppx],[0,fy,ppy],[0,0,1]])

        # フィルターインスタンス取得
        dic_filter = self.get_filter()
        align = dic_filter["align"]

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

                frameNum += 1
                fpscnt += 1
                if fpscnt >= 2:
                    cur_image = color_numpy
                    # print("\t", "frame number : (color,depth) = ( ", color_frame.get_frame_number(), ",",depth_frame.get_frame_number())
                    fpscnt = 0
                    desc = "\\" + str(frameNum)

                    #フレーム保存
                    np.savetxt(pathCSV + desc + ".csv", depth_numpy, delimiter=',', fmt='%10f')
                    cv2.imwrite(pathImg + desc + ".png", cv2.cvtColor(color_numpy, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_PNG_COMPRESSION, 0])
                    if bfr_image is not None:
                        kpts0, kpts1 = self.match.extract_matching(bfr_image, cur_image)
                        ret = self.pose.estimate_pose(kpts0, kpts1, K, K)

                        if ret is None:
                            err_t, err_R = np.inf, np.inf
                            print("カメラ姿勢推定失敗",err_t, err_R)
                            continue
                        else:
                            R, t, inliers = ret
                            print(R,t)
                            exit()

                    bfr_image = cur_image

            print("finish")
        finally:
            pipeline.stop()


if __name__ == '__main__':
    bag_ctl = bag_ctl()
    bag_ctl.extractFrameFromBag(bagfilepath=r"C:\Users\dobashi\Documents\20210204_152209.bag",
                        pathCSV = r"D:\test\aaaaaa\depth",
                        pathImg = r"D:\test\aaaaaa\img")