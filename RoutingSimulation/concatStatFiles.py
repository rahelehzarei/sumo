import xlrd
import os
import glob
import pandas as pd
import numpy as np
import openpyxl
import sys
# import sumolib
import re


dirpath = os.getcwd()
algoPath = 'RoutingSim' #sys.argv[1]
algo =sys.argv[1]
isRerouting =int(sys.argv[2])
TripsNo =sys.argv[3]
statdata = glob.glob(dirpath+'/'+ algoPath+ algo+ '/'+TripsNo +'-statistics*.csv')
statdata.sort(key=lambda f: int(re.sub('\D', '', f)))

all_data = pd.DataFrame()
for f in statdata:
    df = pd.read_csv(f)
    all_data = all_data.append(df,ignore_index=True)

all_data =pd.DataFrame(all_data, columns=["totalVeh","totalTravelTime","totalTravelLength","totalDepartDelay","totalWaitTime","avgTravelTime","avgTravelLength","avgTravelSpeed","avgDepartDelay"])
all_data['netNo'] = all_data.index+1

log_files = glob.glob(dirpath+'/'+ algoPath+ algo+ '/'+TripsNo +'-log*.txt')
log_files.sort(key=lambda f: int(re.sub('\D', '', f)))

loglist=[]
for l in log_files:
    f = open(l, 'r').read()
    total_veh = float(f.partition('Inserted: ')[2].split(' ')[0].partition('\n')[0])
    # print (f.partition('Inserted: ')[2].split(' ')[0].partition('\n')[0])
    netNo = f.partition('RoutingSim')[2].split('.net.xml')[0]
    explored_edge = float( f.partition('explored ')[2].split(' ')[0])
    spentTime_query = float (f.partition('spent ')[2].split(' ')[0].split('ms')[0])
    duration = float(f.partition('Duration: ')[2].split(' ')[0].split('ms\n')[0])
    simulationTime =float(f.partition('Simulation ended at time: ')[2].split(' ')[0].split('\nReason')[0])
    RouteLength = float(f.partition('RouteLength: ')[2].split(' ')[0].split('\n')[0])
    # UPS= float(f.partition('UPS: ')[2].split(' ')[0].split('\nVehicles')[0])
    waitingTime= float(f.partition('WaitingTime: ')[2].split(' ')[0].split('\n')[0])
    TimeLoss = float(f.partition('TimeLoss: ')[2].split(' ')[0].split('\n')[0])
    loglist.append([total_veh,netNo,explored_edge,spentTime_query, duration,simulationTime,RouteLength,waitingTime])

log_data = pd.DataFrame(loglist,columns=["totalVeh","NETNO","explored_edge","spentTime_query","duration","simulationTime","RouteLength","waitingTime"])
log_data['netNo'] = log_data.index + 1

df_merge_col = pd.merge(all_data, log_data, on='netNo')



if isRerouting:
    df_merge_col.to_excel(dirpath+ '/' + algoPath +algo+ '/'+ TripsNo +'-totalStat'+algo+'Rerouting.xlsx', sheet_name=TripsNo+'-'+str(algo)+'_rerouting')
else:
    df_merge_col.to_excel(dirpath + '/' + algoPath + algo +'/'+ TripsNo + '-totalStat' + algo + '.xlsx', sheet_name=TripsNo+'-'+str(algo))