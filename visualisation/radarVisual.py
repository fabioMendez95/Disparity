#Used to Visualize the data points given by the radar and read by the
#Main program executed on CUDA
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math

class visualRadar:
    path = "/tmp/FIFO"
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)

    def readPipe(self): 
        fifo = open(self.path,'r')
        received = fifo.read()
        return received

    #Points need send in the following format "PointNumber \n x1 y1 \n x2 y2 "
    def getArray(self, data):
        infoFromProgram = data.split("\n")
        pointNum = int(infoFromProgram[0])
        pointsStr = (infoFromProgram[1:])
        pointArray = np.zeros(shape = (pointNum,2))
    
        for pointPoss in range(0,pointNum):
            splitInfo = pointsStr[pointPoss].split(' ')
            pointArray[pointPoss,0] = float(splitInfo[0])
            #np.arctan(float(splitInfo[1])/float(splitInfo[0]))
            pointArray[pointPoss,1] = float(splitInfo[1])
            #math.sqrt(float(splitInfo[1])*float(splitInfo[1]) + float(splitInfo[0])*float(splitInfo[0]))

        return pointArray

    def plotPoints(self, pointArray):
        plt.xlim(-5,5)
        plt.ylim(0,10)
        self.ax.scatter(pointArray[:,0],pointArray[:,1])
        plt.show()
        self.fig.canvas.draw()
        plt.cla()
    
    def visualisation(self):
        while True:
            try:
                receivedData = self.readPipe()
                infoPoints = self.getArray(receivedData)
                print infoPoints
                self.plotPoints(infoPoints)
            except:
                time.sleep(0.2)#Needs to be smaller than the sampling rate of the Radar for better visualisation     
                continue

radar = visualRadar()
radar.visualisation()
