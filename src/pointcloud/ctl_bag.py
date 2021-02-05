import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import match_superglue as mt
import pose_estimate as ps

class bag_ctl():


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
        # フィルターインスタンス取得
        dic_filter = self.get_filter()
        align = dic_filter["align"]

        fpscnt = 0
        frameNum = 0
        bfr_image = None
        cur_image = None

        try:
            # マッチングクラス
            match = mt.match_glue()
            # カメラ姿勢クラス
            pose = ps.BaseOpenCV()

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

                    np.savetxt(pathCSV + desc + ".csv", depth_numpy, delimiter=',', fmt='%10f')
                    cv2.imwrite(pathImg + desc + '.png', color_numpy, [cv2.IMWRITE_PNG_COMPRESSION, 0])

                    fx = color_intrinsics.fx
                    fy = color_intrinsics.fy
                    ppx = color_intrinsics.ppx
                    ppy = color_intrinsics.ppy

                    if bfr_image is not None:
                        device = "cpu"
                        cur_image = cv2.cvtColor(cur_image, cv2.COLOR_BGR2GRAY)
                        bfr_image = cv2.cvtColor(bfr_image, cv2.COLOR_BGR2GRAY)
                        kpts0, kpts1 = match.extract_matching(bfr_image, cur_image)

                        print(kpts0.shape, kpts1.shape)
                        pose
                        exit()

                        # # bfr_image = cv2.imread(in1, cv2.IMREAD_GRAYSCALE)
                        # # cur_image = cv2.imread(in0, cv2.IMREAD_GRAYSCALE)
                        #
                        # device = "cpu"
                        # rot0 = 0
                        # # h=bfr_image.shape[0]
                        # # w=bfr_image.shape[1]
                        # h = 640
                        # w = 480
                        #
                        # image0, inp0 = mt.read_image(bfr_image, device)
                        # image1, inp1 = mt.read_image(cur_image, device)
                        # mt.matching(inp0, inp1)
                        # exit()
                        #
                        # in1 = r"D:\test\git\practice_algorithm\src\pointcloud\otherlib\SuperGluePretrainedNetwork\assets\scannet_sample_images\scene0726_00_frame-000135.jpg"
                        # in0 = r"D:\test\git\practice_algorithm\src\pointcloud\otherlib\SuperGluePretrainedNetwork\assets\scannet_sample_images\scene0726_00_frame-000210.jpg"
                        #
                        # print(in0, device, [h, w], rot0, False)
                        #
                        # image0, inp0, scales0 = mt.read_image(cur_image, "cpu")
                        # image1, inp1, scales1 = mt.read_image(bfr_image, "cpu")
                        # mt.matching(inp0, inp1)
                        #
                        # exit()
                        #
                        # image0, inp0 = mt.read_image(bfr_image, device)
                        # image1, inp1 = mt.read_image(cur_image, device)
                        #
                        # mt.matching(inp0, inp1)
                        # exit()
                    bfr_image = cur_image

            print("finish")
        finally:
            pipeline.stop()


if __name__ == '__main__':
    bag_ctl = bag_ctl()
    bag_ctl.extractFrameFromBag(bagfilepath=r"C:\Users\dobashi\Documents\20210204_152209.bag",
                        pathCSV = r"D:\test\新しいフォルダー\depth",
                        pathImg = r"D:\test\新しいフォルダー\img")