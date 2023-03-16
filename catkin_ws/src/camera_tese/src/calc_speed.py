import cv2
import numpy as np

class Calc_Speed:

    def CalcSpeed(self, dist, ponto_atual, pts):

        print("ponto atual= ", ponto_atual)
        #print("pts= ", pts)

        if ponto_atual == 0:
            dist = 0
        else:
            dist = pts[ponto_atual] - pts[ponto_atual - 1]
        
        print("dist= ", dist)
