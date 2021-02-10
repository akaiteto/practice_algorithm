import cv2
import numpy as np
import utils_pcl as util_3d
import utils_image as util_2d

class Evaluate():
    class Reproject_3Dto2D():
        def Evaluate(self,pcl1,color0,K,flgSave=False):
            imgM_T2M, imgM_hole_on_T = util_3d.Reprojection_3Dto2D(pcl_target=pcl1, img_main=color0, K=K)
            dif_M = util_2d.DiffImage(imgM_hole_on_T.copy(), imgM_T2M.copy())
            if flgSave:
                cv2.imwrite("imgM_T2M.png", cv2.cvtColor(imgM_T2M, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_PNG_COMPRESSION, 0])
                cv2.imwrite("imgM_hole_on_T.png", cv2.cvtColor(imgM_hole_on_T, cv2.COLOR_BGR2RGB),
                            [cv2.IMWRITE_PNG_COMPRESSION, 0])
                cv2.imwrite('M_Diff.jpg', dif_M)

            difcntM = np.count_nonzero(dif_M != 0)
            wholeM = np.count_nonzero(imgM_hole_on_T != 0)
            ratioM = difcntM / wholeM

            return ratioM

    class maskfindEmat():
        def Evaluate(self,pcl_target, P_target,img_main,K):
            '''
            essential行列より評価
            '''
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

