import csv
import os
import utils_pcl as util_3d
import numpy as np
import cv2
import pose_estimate as ps
import eval_registration as ev

class known_pose():
    def integrate_3D(self, pathOutput, dicView, K,idx_initView=0):
        '''
        カメラ姿勢から各視点の点群を統合する
        '''
        imgM = cv2.imread(dicView[idx_initView]["img"])
        imgM_T2M = np.zeros(imgM.shape, dtype=np.uint8)

        for key in dicView.keys():
            print(dicView[key]["ply"])
            FileNm = os.path.basename(dicView[key]["ply"])
            pcl = util_3d.read_point_cloud(dicView[key]["ply"])
            P = np.loadtxt(dicView[key]["pose"], delimiter=',', dtype='float64')
            pcl_trans = util_3d.ApplyPmat4x4(pcl, P)
            util_3d.write_point_cloud(os.path.join(pathOutput, FileNm), pcl_trans)

    def integrate_2D(self,pathOutput,dicView,K,idx_initView=0):
        '''
        カメラ姿勢から各視点の画像を1つの画像に変換する
        '''
        imgM = cv2.imread(dicView[idx_initView]["img"])
        imshape = (imgM.shape[0],imgM.shape[1]*4,imgM.shape[2])
        imgM_T2M = np.zeros(imshape, dtype=np.uint8)

        for key in dicView.keys():
            print(dicView[key]["ply"])
            FileNm = os.path.basename(dicView[key]["ply"])
            pcl = util_3d.read_point_cloud(dicView[key]["ply"])
            P = np.loadtxt(dicView[key]["pose"], delimiter=',', dtype='float64')
            pcl_trans = util_3d.ApplyPmat4x4(pcl, P)
            # util_3d.write_point_cloud(os.path.join(pathOutput, FileNm), pcl_trans)

            imgM_T2M, imgM_hole_on_T = util_3d.Reprojection_3Dto2D(pcl_target=pcl_trans, img_main=imgM, K=K,imgM_T2M=imgM_T2M)
            cv2.imwrite("integrate2D.png", cv2.cvtColor(imgM_T2M, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_PNG_COMPRESSION, 0])

    def getDepthCSPerView(self, path):
        # depthデータ取得
        in_csvPath = path
        with open(in_csvPath) as f:
            reader = csv.reader(f)
            csvlist = [row for row in reader]

        return csvlist

    def refine(self,pathOutput,dicView,K,idx_initView=0):
        '''
        カメラ姿勢から各視点の画像を1つの画像に変換する
        '''
        imgM = cv2.imread(dicView[idx_initView]["img"])
        depthM = np.array(self.getDepthCSPerView(dicView[idx_initView]["depth"]))
        pcl_M = util_3d.read_point_cloud(dicView[idx_initView]["ply"])
        P_Main = np.loadtxt(dicView[idx_initView]["pose"], delimiter=',', dtype='float64')
        pose = ps.PoseEstimation.Common()
        eval = ev.Evaluate.maskfindEmat()

        for key in dicView.keys():
            if key==idx_initView:continue
            print("pair:( ",idx_initView," , ",key," )")
            pclT = util_3d.read_point_cloud(dicView[key]["ply"])
            imgT = cv2.imread(dicView[key]["img"])
            depthT = np.array(self.getDepthCSPerView(dicView[key]["depth"]))
            P_target = np.loadtxt(dicView[key]["pose"], delimiter=',', dtype='float64')
            # eval.Evaluate(pcl_target = pcl_target, P_target = P_target,img_main=imgM,K=K)
            pose.refinePose(pcl_target = pclT, P_target = P_target,img_main=imgM,img_target=imgT,depthT=depthT,depthM=depthM,K=K)


    def integrate(self,pathOutput,dicView,K,idx_initView=0):
        '''

        '''
        K=np.array(K,dtype = np.float32)
        imgM = cv2.imread(dicView[idx_initView]["img"])
        depthM = np.array(self.getDepthCSPerView(dicView[idx_initView]["depth"]),dtype = np.float32)
        pose = ps.PoseEstimation.Common()

        FileNm_M ="raw_" + str(idx_initView)+ ".ply"
        util_3d.generate_pointcloud(fW=K[0][0], fH=K[1][1], centerW=K[0][2], centerH=K[1][2],
                                    rgb=imgM, depth=depthM,
                                    ply_file=os.path.join(pathOutput, FileNm_M))
        pclM = util_3d.read_point_cloud(os.path.join(pathOutput, FileNm_M))

        for key in dicView.keys():
            if key==idx_initView:continue
            imgT = cv2.imread(dicView[key]["img"])
            depthT = np.array(self.getDepthCSPerView(dicView[key]["depth"]),dtype = np.float32)

            print("pair:( ", idx_initView, " , ", key, " )")
            FileNm_T = "raw_" + str(key) + ".ply"
            util_3d.generate_pointcloud(fW=K[0][0], fH=K[1][1], centerW=K[0][2], centerH=K[1][2],
                                        rgb=imgT, depth=depthT,
                                        ply_file=os.path.join(pathOutput, FileNm_T))
            pclT = util_3d.read_point_cloud(os.path.join(pathOutput, FileNm_T))

            success, trans, _ = pose.integrate(img_main=imgT,img_target=imgM,depthT=depthM,depthM=depthT,K=K)
            if success:
                FileNm_T = "trans_" + str(idx_initView) + "_" + str(key) + ".ply"
                pclT_trans = util_3d.ApplyPmat4x4(pclT, trans)
                util_3d.write_point_cloud(os.path.join(pathOutput, FileNm_T), pclT_trans)
            else:
                print("fail :( ",idx_initView," , ",key," )")
                imgM=imgT
                depthM = depthT
                idx_initView=key



    def converView(self,pathOutput,dicView,dicTargetView,K):
        P_tar = np.loadtxt(dicTargetView["pose"], delimiter=',', dtype='float64')
        R_tar,t_tar = util_3d.ConvP2RT(P_tar)
        R_inv,t_inv =  util_3d.getInverseRt(R_tar,t_tar)
        P_inv_tar = util_3d.ConvRT2TransXYZW(R_inv,t_inv.reshape(3))

        imgM = cv2.imread(dicTargetView["img"])
        imshape = (imgM.shape[0],imgM.shape[1],imgM.shape[2])
        imgM_T2M = np.zeros(imshape, dtype=np.uint8)

        for key in dicView.keys():
            pcl = util_3d.read_point_cloud(dicView[key]["ply"])
            P = np.loadtxt(dicView[key]["pose"], delimiter=',', dtype='float64')
            P = np.dot(P,P_inv_tar)
            _, t = util_3d.ConvP2RT(P)
            dist_target =np.linalg.norm(t)

            pcl_trans = util_3d.ApplyPmat4x4(pcl, P)
            imgM_T2M, imgM_hole_on_T = util_3d.Reprojection_3Dto2D(pcl_target=pcl_trans, img_main=imgM, K=K,imgM_T2M=imgM_T2M)

            cnt_success = np.count_nonzero(imgM_T2M != 0)
            cnt_whole = np.count_nonzero(imgM != 0)
            ratio = cnt_success/cnt_whole
            if ratio > 0.8:break
            if dist_target > 0.5:break
        cv2.imwrite(os.path.join(pathOutput, "integrate_OtheView.png"), cv2.cvtColor(imgM_T2M, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_PNG_COMPRESSION, 0])
        cv2.imwrite(os.path.join(pathOutput, "Original.png"), cv2.cvtColor(imgM, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_PNG_COMPRESSION, 0])


if __name__ == '__main__':
    dicView = {}
    K=np.array([[913.49,0,639.614],[0,913.49,372.977],[0,0,1]])
    pathImg = r"D:\test\L177\img"
    pathPose = r"D:\test\L177\pose"
    pathInput = r"D:\test\L177\input"
    pathDepth = r"D:\test\L177\depth"
    pathOutput_PLY = r"D:\test\L177\output\integrate"
    pathOutput_VIEWCONV = r"D:\test\L177\output\viewconv"

    #視点変換したい視点の情報を格納
    i_target = 3
    s = str(i_target + 1)
    s_zero = s.zfill(10)
    FmImg = s_zero + ".png"
    FmDepth = s_zero + ".csv"
    FmInp = str(i_target) + "_Camera.ply"
    FmPose = str(i_target) + "_transform.txt"
    dicTargetView = {"img":os.path.join(pathImg, FmImg),
                    "pose":os.path.join(pathPose, FmPose),
                    "ply":os.path.join(pathInput, FmInp),
                    "depth": os.path.join(pathDepth, FmDepth)}

    #全視点の視点情報を格納
    for i in range(97):
        if i==i_target:continue #ターゲットの画像除く
        s = str(i+1)
        s_zero = s.zfill(10)
        FmImg = s_zero + ".png"
        FmInp = str(i)+"_Camera.ply"
        FmPose = str(i)+"_transform.txt"
        FmDepth = s_zero + ".csv"

        flg = os.path.exists(os.path.join(pathImg, FmImg)) and os.path.exists(os.path.join(pathPose, FmPose)) and os.path.exists(os.path.join(pathInput, FmInp))
        if flg:
            dicView[i] = {"img":os.path.join(pathImg, FmImg),
                         "pose":os.path.join(pathPose, FmPose),
                         "ply":os.path.join(pathInput, FmInp),
                         "depth": os.path.join(pathDepth, FmDepth)}


    KnownPose = known_pose()
    # KnownPose.integrate_3D(pathOutput=pathOutput_PLY,dicView=dicView,K=K)
    # KnownPose.integrate_2D(pathOutput=pathOutput_PLY,dicView=dicView,K=K)
    # KnownPose.converView(pathOutput=pathOutput_VIEWCONV, dicView=dicView,dicTargetView=dicTargetView, K=K)
    # KnownPose.refine(pathOutput=pathOutput_PLY,dicView=dicView,K=K)
    KnownPose.integrate(pathOutput=pathOutput_PLY, dicView=dicView, K=K)