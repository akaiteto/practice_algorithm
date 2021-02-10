import cv2
import numpy as np
import random

def getMatchImage(srcImg,dstImg,srcPts,dstPts):
    im_concat = cv2.hconcat([srcImg,dstImg])
    height=srcImg.shape[0]
    width=srcImg.shape[1]
    for idx in range(len(srcPts)):
        spt_w,spt_h = int(srcPts[idx][0]),int(srcPts[idx][1])
        tpt_w,tpt_h = int(dstPts[idx][0]),int(dstPts[idx][1])
        cv2.line(im_concat, (spt_w,spt_h),(tpt_w+width, tpt_h),(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)), thickness=10, lineType=cv2.LINE_4)
    return im_concat

def DiffImage(imgM, imgT):
    imgM = cv2.cvtColor(imgM, cv2.COLOR_BGR2GRAY)
    imgT = cv2.cvtColor(imgT, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    imgM = clahe.apply(imgM)
    imgT = clahe.apply(imgT)

    fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()

    fgmask = fgbg.apply(imgM)
    fgmask = fgbg.apply(imgT)

    return fgmask