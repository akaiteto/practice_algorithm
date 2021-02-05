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

