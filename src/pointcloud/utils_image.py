import cv2
import numpy as np

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