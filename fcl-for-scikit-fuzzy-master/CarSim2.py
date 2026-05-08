import numpy as np
import cv2
import math

def vectorLength(x, y):
    return math.sqrt(x*x + y*y)

def alpha(P1, P2): #P2--> P1
    x = P1[0] - P2[0]
    y = P1[1] - P2[1]
    return math.atan( y / x)
    
    
def rotatePoint(x, y,  cx, cy, a):
    x= x - cx 
    y= y - cy
    return (cx + int( x*math.cos(a) - y * math.sin(a)), cy + int( x*math.sin(a) + y * math.cos(a) ) )

def coordinateMapping(x, y, maxY=512):
    return (x, maxY-y)

class Simulation():
    def __init__(self, ox=800, oy=256, scrsize=512):
        # Screen Size
        self.SCREEN_SIZE = scrsize

        # ParkingLot Parameters
        self.SCREEN_CENTER_X = int( self.SCREEN_SIZE / 2 )
        self.SCREEN_CENTER_Y = int( self.SCREEN_SIZE / 2 )

        self.parkingLotWidth = 250
        self.parkingLotHeight = 150
        self.Tip1_X = self.SCREEN_CENTER_X + int(0.5*self.parkingLotWidth)
        self.Tip1_Y = 0
        self.Tip2_X = self.SCREEN_CENTER_X - int(0.5*self.parkingLotWidth)
        self.Tip2_Y = 0
        self.Tip3_X = self.SCREEN_CENTER_X + int(0.5*self.parkingLotWidth)
        self.Tip3_Y = self.parkingLotHeight 
        self.Tip4_X = self.SCREEN_CENTER_X - int(0.5*self.parkingLotWidth)
        self.Tip4_Y = self.parkingLotHeight 

        ## Car Parameters
        self.theta = 0
        self.phi = 0.0     # Handler Angles
        self.velocity= 0.5      # car velocity 
        self.l_axis = 150     # length of wheel axis
        self.carWidth = 80    # car width
        self.WheelLength = 15 
        self.ext = 10
        self.carLength = self.l_axis + self.ext * 2

        self.OrgXr= ox
        self.OrgYr= oy #int(self.SCREEN_SIZE / 2)
        self.Xr = self.OrgXr
        self.Yr = self.OrgYr
        self.Xf = 0
        self.Yf = 0
        
    def getTheta(self):
        return self.theta
        
    def getRearToLotTip1(self):  # return (DistXr2Tip1)
        return ( self.Xr - self.Tip1_X)
        
    def getRearToLotTip2(self):
        return ( self.Xr - self.Tip2_X)
    
    def getRearToLotTip3(self):
        return ( self.Xr - self.Tip3_X)
    
    def getRearToLotTip4(self):
        return ( self.Xr - self.Tip4_X)
    
    def getFrontToLotTip1(self):
        return ( self.Xf - self.Tip1_X)
        
    def getFrontToLotTip2(self):
        return ( self.Xf - self.Tip2_X)


    def getFrontToLotTip3(self):
        return ( (self.Tip3_X-self.Xf, self.Tip3_Y - self.Yf), alpha( (self.Tip3_X, self.Tip3_Y), (self.Xf, self.Yf) ) )
        
    def getFrontToLotTip4(self):
        return ( (self.Tip4_X-self.Xf, self.Tip4_Y - self.Yf), alpha( (self.Tip4_X, self.Tip4_Y), (self.Xf, self.Yf) ) )

    def getCurrentPhi(self):
        return ( self.phi )

    def drawSimulation(self, phi, velocity):
        self.phi = phi
        self.velocity= velocity 


        #compute four points of the car when it is still and not rotated relative the center
        pLeftX = int( -0.5 * self.l_axis - self.ext )
        pRightX = int( 0.5 * self.l_axis + self.ext )
        pTopY =  int(0.5 * self.carWidth)
        pBottomY = - int(0.5 * self.carWidth)

        # compute parameters
        dTheta = self.velocity * math.sin(self.phi) / self.l_axis
        self.theta += dTheta
        deltaXr = self.velocity * math.cos(self.theta) * math.cos(self.phi)
        deltaYr = self.velocity * math.sin(self.theta) * math.cos(self.phi)
        self.Xr = int(self.Xr + deltaXr)
        self.Yr = int(self.Yr + deltaYr)
        self.Xf = int(self.Xr + self.l_axis * math.cos(self.theta) )
        self.Yf = int(self.Yr + self.l_axis * math.sin(self.theta) )

        #compute the car box
        carCenterX = int ( (self.Xr + self.Xf) / 2)
        carCenterY = int ( (self.Yr + self.Yf) / 2)

        p1_x, p1_y = np.add((pLeftX, pTopY) , (carCenterX, carCenterY) )
        p1_x, p1_y = rotatePoint(p1_x, p1_y, carCenterX, carCenterY,  self.theta)

        p2_x, p2_y = np.add((pRightX, pTopY) , (carCenterX, carCenterY) )
        p2_x, p2_y = rotatePoint(p2_x, p2_y,carCenterX, carCenterY,  self.theta)

        p3_x, p3_y = np.add((pRightX, pBottomY) , (carCenterX, carCenterY) )
        p3_x, p3_y = rotatePoint(p3_x, p3_y, carCenterX, carCenterY, self.theta)

        p4_x, p4_y = np.add((pLeftX, pBottomY) , (carCenterX, carCenterY) )
        p4_x, p4_y = rotatePoint(p4_x, p4_y,carCenterX, carCenterY, self.theta) 

        #compute the wheels boxs (thick lines)
        #後輪1,2
        w1_x1, w1_y1 =  np.add((pLeftX, pTopY) , (carCenterX, carCenterY) )
        w1_x1, w1_y1 =  rotatePoint(w1_x1, w1_y1 ,carCenterX, carCenterY, self.theta)

        w1_x2, w1_y2 = np.add((pLeftX+self.WheelLength, pTopY) , (carCenterX, carCenterY) )
        w1_x2, w1_y2 = rotatePoint(w1_x2, w1_y2, carCenterX, carCenterY, self.theta) 

        w2_x1, w2_y1 = np.add((pLeftX, pBottomY) , (carCenterX, carCenterY) )
        w2_x1, w2_y1 =rotatePoint(w2_x1, w2_y1 , carCenterX, carCenterY,self.theta)

        w2_x2, w2_y2 = np.add((pLeftX+self.WheelLength, pBottomY) , (carCenterX, carCenterY) )
        w2_x2, w2_y2 = rotatePoint(w2_x2, w2_y2 , carCenterX, carCenterY,self.theta)

        #前輪1

        w3_x1, w3_y1 = np.add((pRightX, pTopY) , (carCenterX, carCenterY) ) #current wheel end points
        w3_x1, w3_y1 =  rotatePoint(w3_x1, w3_y1, carCenterX, carCenterY, self.theta)  #rotate around the car center

        w3_x2, w3_y2 = np.add((pRightX-self.WheelLength, pTopY) , (carCenterX, carCenterY) )#current wheel end points
        w3_x2, w3_y2 =  rotatePoint(w3_x2, w3_y2, carCenterX, carCenterY, self.theta) #rotate around the wheel center

        wCenterX, wCenterY =( int(0.5*(w3_x1+w3_x2)) , int( 0.5 * (w3_y1 + w3_y2)) ) #current center of the wheel without rotation
        w3_x1, w3_y1 =  rotatePoint(w3_x1, w3_y1, wCenterX, wCenterY, self.phi)  #rotate around the wheel center
        w3_x2, w3_y2 =  rotatePoint(w3_x2, w3_y2, wCenterX, wCenterY, self.phi) #rotate around the wheel center


        #前輪2
        w4_x1, w4_y1 = np.add((pRightX, pBottomY) , (carCenterX, carCenterY) ) #current wheel end points
        w4_x1, w4_y1 = rotatePoint(w4_x1, w4_y1,  carCenterX, carCenterY, self.theta)

        w4_x2, w4_y2 = np.add((pRightX-self.WheelLength, pBottomY) , (carCenterX, carCenterY) ) #current wheel end points
        w4_x2, w4_y2 = rotatePoint(w4_x2, w4_y2, carCenterX, carCenterY, self.theta)

        wCenterX, wCenterY =( int(0.5*(w4_x1+w4_x2)) , int( 0.5 * (w4_y1 + w4_y2)) ) #current center of the wheel without rotation

        w4_x1, w4_y1 = rotatePoint(w4_x1, w4_y1,  wCenterX, wCenterY, self.phi) #rotate around the wheel center
        w4_x2, w4_y2 = rotatePoint(w4_x2, w4_y2, wCenterX, wCenterY, self.phi) #rotate around the wheel center

        # coordinateMapping all points to draw on the screen
        sp1_x, sp1_y = coordinateMapping(p1_x, p1_y)
        sp2_x, sp2_y = coordinateMapping(p2_x, p2_y)
        sp3_x, sp3_y = coordinateMapping(p3_x, p3_y)
        sp4_x, sp4_y = coordinateMapping(p4_x, p4_y)

        sXr, sYr = coordinateMapping(self.Xr, self.Yr)
        sXf, sYf = coordinateMapping(self.Xf, self.Yf)

        sw1_x1, sw1_y1 = coordinateMapping(w1_x1, w1_y1)
        sw1_x2, sw1_y2 = coordinateMapping(w1_x2, w1_y2)

        sw2_x1, sw2_y1 = coordinateMapping(w2_x1, w2_y1)
        sw2_x2, sw2_y2 = coordinateMapping(w2_x2, w2_y2)

        sw3_x1, sw3_y1 = coordinateMapping(w3_x1, w3_y1)
        sw3_x2, sw3_y2 = coordinateMapping(w3_x2, w3_y2)

        sw4_x1, sw4_y1 = coordinateMapping(w4_x1, w4_y1)
        sw4_x2, sw4_y2 = coordinateMapping(w4_x2, w4_y2)

        # 建立一張 512x512 的 RGB 圖片（黑色）
        img = np.zeros((self.SCREEN_SIZE, self.SCREEN_SIZE+300, 3), np.uint8)

        # 將圖片用淺灰色 (200, 200, 200) 填滿
        img.fill(200)

        # 畫停車格
        cv2.rectangle(img, (self.SCREEN_CENTER_X- int(0.5*self.parkingLotWidth),
                            self.SCREEN_SIZE-self.parkingLotHeight),
                      (self.SCREEN_CENTER_X+int(0.5*self.parkingLotWidth), self.SCREEN_SIZE-1),
                      (255, 255, 255), 5)

        # 畫車子
        # 設定Car多邊形頂點座標
        pts = np.array([[sp1_x, sp1_y], [sp2_x, sp2_y], [sp3_x, sp3_y], [sp4_x, sp4_y]], np.int32)

        # 將座標轉為 (頂點數量, 1, 2) 的陣列
        pts = pts.reshape((-1, 1, 2))

        # 繪製多邊形
        cv2.polylines(img, [pts], True, (255, 0, 0), 4)

        # 繪製軸距
        cv2.line(img, (sXr, sYr), (sXf, sYf), (0, 0, 255), 5)

        # 繪製輪胎
        cv2.line(img, (sw1_x1, sw1_y1), (sw1_x2, sw1_y2), (0, 255, 0), 3)
        cv2.line(img, (sw2_x1, sw2_y1), (sw2_x2, sw2_y2), (0, 255, 0), 3)
        cv2.line(img, (sw3_x1, sw3_y1), (sw3_x2, sw3_y2), (0, 255, 0), 3)
        cv2.line(img, (sw4_x1, sw4_y1), (sw4_x2, sw4_y2), (0, 255, 0), 3)
        ########################################


        # 顯示圖片
        cv2.imshow('Car Simulation', img)

        # 按下任意鍵則關閉所有視窗
        cv2.waitKey(100)
        #cv2.destroyAllWindows()

if __name__ == '__main__':
    simulator =  Simulation(400, 256)
    phi=0
    v=0
    stop = False
    #for i in range(40):
    phase = 1
    while (not stop):
        # phi = 0 # -0.7
        
        simulator.drawSimulation(phi, v) # phi, velocity
        if phase == 1:
            v += -0.1 #倒退的速度
           # ((x, y), a) = simulator.getRearToLotTip1()
            x = simulator.Xr
            print(x)
            if  abs( simulator.getRearToLotTip1()) < 10:    #abs(x - (256+150) ) < 10:
                phase = 2
        if  phase == 2:
            phi= -3.14/4
            v = -2.7
            th = simulator.getTheta()
            print(th)
            if abs(th - 3.14/4) < 0.1:
                phase = 3
        if  phase == 3:
            phi = 0
            v = -1.7
            x = simulator.Xr
            print(x)
            if abs( simulator.getRearToLotTip2()) < 100:    # abs(x - (210) ) < 10:
                phase = 4
        if  phase == 4:
            phi= 3.14/4
            v = -1.7
            th = simulator.getTheta()
            print(th)
            if abs(th - 0) < 0.1:
                phase= 5

        if  phase == 5:
            phi= -0.1
            v = 1.5
            th = simulator.getTheta()
            print(th)
            if abs(th - 0) < 0.03:
                stop = True

           
