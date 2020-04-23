import glob
import os
import xlrd
import openpyxl
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import re
from matplotlib.legend_handler import HandlerLine2D

# import natsort

dirpath = os.getcwd()
algoPath = 'RoutingSim'

statFilesDijkstra = glob.glob(dirpath + '/' + algoPath + 'dijkstra' + '/*-totalStatdijkstra.xlsx')
# for f in statFilesDijkstra:
#     print (int(f.split('Trips')[1].partition('-totalStatdijkstra')[0]))
# sorted(statFilesDijkstra,key=lambda f: int(f.split('Trips')[1].partition('-totalStatdijkstra')[0]))
statFilesDijkstra.sort(key=lambda f: int(re.sub('\D', '', f)))
# print(statFilesDijkstra)
statFilesAstar = glob.glob(dirpath + '/' + algoPath + 'astar' + '/*-totalStatastar.xlsx')
statFilesAstar.sort(key=lambda f: int(re.sub('\D', '', f)))
statFilesCH = glob.glob(dirpath + '/' + algoPath + 'CH' + '/*-totalStatCH.xlsx')
statFilesCH.sort(key=lambda f: int(re.sub('\D', '', f)))



statDataDijkstra = []
for f in statFilesDijkstra:
    df = pd.read_excel(f)
    # pd.to_numeric(df['NETNO'])    # print(df)
    dij = np.mean(df['avgTravelLength'])
    statDataDijkstra.append(dij)

statDataAstar = []
# print(statFilesAstar)
for f in statFilesAstar:
    df = pd.read_excel(f)
    # pd.to_numeric(df['NETNO'])
    astar = np.mean(df['avgTravelLength'])
    statDataAstar.append(astar)


statDataCH = []
for f in statFilesCH:
    df = pd.read_excel(f)
    # pd.to_numeric(df['NETNO'])
    ch = np.mean(df['avgTravelLength'])
    statDataCH.append(ch)

allData = pd.DataFrame([statDataDijkstra, statDataAstar, statDataCH],
            columns=["10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "110" , "120", "130", "140", "150", "160", "170", "180", "190", "200"])

# allData.astype(int)
print(allData)
allData.T.plot()
plt.xlabel("Number of trips")
plt.ylabel("avg Travel Length")
plt.legend(['dijkstra' , 'astar', 'ch'])
plt.show()
plt.savefig('avgTravelLength_100.png')
