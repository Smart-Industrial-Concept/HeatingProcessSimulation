# -*- coding: utf-8 -*-
"""
Created on Thu Jul  2 12:28:51 2020

@author: guser
"""

import math as m
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math as m
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


#Operation modes
modes={'Off', 'IDLE', 'LEVEL1', 'LEVEL2'}

#temperature [Kelvin], fan_in pressure , fan_out pressure  
setpoints={'IDLE':[30+273.15,200,5],
         'LEVEL1':[50+273.15,300,5],
         'LEVEL2':[70+273.15,300,5]}


timeIndex=pd.date_range(start='2020-04-15', end="2020-04-17 23:00", freq='Min')

#%% Load 1
def generateExponentialLoad(length,A,T):
    #A=-1000
    #T=50
    t=np.arange(0,length)
    y=A*np.exp(-t/T)
    return y

#%% define profile for a workday
loadData=pd.DataFrame(0, index=timeIndex, columns=['Load'])
inputData=pd.DataFrame(0,index=timeIndex,columns=['WorkingPoint','Setpoint_Tp','Setpoint_Fan_in','Setpoint_Fan_out'])
inputData['WorkingPoint']='OFF'

#generate the profile
currentMode='IDLE'
inputData.loc[inputData.between_time('06:00:00' , '08:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
currentMode='LEVEL1'
inputData.loc[inputData.between_time('08:00:00' , '09:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
loadData['Load']['2020-04-15 8:15':'2020-04-15 08:45']=generateExponentialLoad(len(inputData['2020-04-15 8:15':'2020-04-15 08:45']),-500,15)

currentMode='IDLE'
inputData.loc[inputData.between_time('09:00:00' , '10:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
currentMode='LEVEL2'
inputData.loc[inputData.between_time('10:00:00' , '12:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
loadData['Load']['2020-04-15 10:20':'2020-04-15 11:55']= generateExponentialLoad(len(inputData['2020-04-15 10:20':'2020-04-15 11:55']),-1000,50)

currentMode='IDLE'
inputData.loc[inputData.between_time('12:00:00' , '14:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
currentMode='LEVEL1'
inputData.loc[inputData.between_time('14:00:00' , '15:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
loadData['Load']['2020-04-15 14:15':'2020-04-15 14:45']=generateExponentialLoad(len(inputData['2020-04-15 14:15':'2020-04-15 14:45']),-400,20)

currentMode='IDLE'
inputData.loc[inputData.between_time('15:00:00' , '16:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
currentMode='LEVEL2'
inputData.loc[inputData.between_time('16:00:00' , '18:00:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]
loadData['Load']['2020-04-15 16:20':'2020-04-15 17:45']=generateExponentialLoad(len(inputData['2020-04-15 16:20':'2020-04-15 17:45']),-900,60)

currentMode='IDLE'
inputData.loc[inputData.between_time('18:00:00' , '18:30:00').index]=[[currentMode,setpoints[currentMode][0],setpoints[currentMode][1],setpoints[currentMode][2]]]

#plot afirst day
inputData[['Setpoint_Tp','Setpoint_Fan_in']]['2020-04-15'].plot()
plt.ylabel('temp [°C] / dp [Pa]')

loadData.plot()
plt.ylabel('load [kW]')
#%% wirte set points to file
FILE_PATH="./inputSetpoints.txt"
inputList=(inputData['Setpoint_Tp'],inputData['Setpoint_Fan_in'],inputData['Setpoint_Fan_out'],loadData['Load'])

#add version string - as required 
tableString="#1\n"

for data in inputList:

    dataLength=len(data.index)
    numIndex=index=np.arange(0,dataLength)

    #add header string
    tableString=tableString +"double " + data.name +" ("+ str(dataLength) +",2)\n"
    
    #create table string: time - value pair
    table=""
    for i in range(dataLength):
        tableString=tableString + str(numIndex[i])+"\t"+str(data.values[i]) + "\n"
    #add 
    
   # tableString=tableString + s
        
f=open(FILE_PATH,'w')
f.write(tableString)
f.close()
