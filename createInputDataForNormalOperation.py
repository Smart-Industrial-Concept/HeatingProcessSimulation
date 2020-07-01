# -*- coding: utf-8 -*-
"""
Created on Tue Jan 28 12:31:57 2020

@author: gsteindl
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
#creat binary random

plt.close('all')

def generatePBRS(duration, minLength=1,offset=0,amplitude=1):
    sym_len=minLength # 1 or 0
    
    no_sym=int(duration/sym_len)
    rand_n=np.random.rand(no_sym)
    
    rand_n[np.where(rand_n >=0.5)]=1
    rand_n[np.where(rand_n < 0.5)]=0
    
    #init signal with zeros
    sig= np.zeros(sym_len*no_sym)
    
    id_n=np.where(rand_n==1)
    
    #create signal
    for i in id_n[0]:
        current_index=int(i*sym_len) 
        sig[current_index:current_index+sym_len]=1
    
    return sig*amplitude+offset

def generateTableString(signal):
    index=list()
    values=list()
    
    index.append(0)
    values.append(sig[0])
    for i in range(1,len(sig)):
        if sig[i] !=sig[i-1]:
            index.append(i)
            values.append(sig[i-1])
            index.append(i)
            values.append(sig[i])
    
    table=""
    for i in range(len(index)):
        table=table + str(index[i])+"\t"+str(values[i]) + "\n"
    
    return table,len(index)
#%%
simulationLength=10000


#generate a table for every input
inputList=('T_inletAir','T_surr', 'setpointFan1_pressure','setpointControl', 'setpointFan2_pressure')
lowBound=(5+273.15,10+273.15,300,20+273.15,5)
amp=(10,5,200,6,-2)
minLength=(100,100,100,100,100)
#tableNames=("T_inlet","T_surr","Fan_pres","T_cont")
signals=list()

tableString="#1\n"

for i in range(len(inputList)):
    
    sig=generatePBRS(simulationLength,minLength[i],lowBound[i],amp[i])
    signals.append(sig)
    
    #add string
    s,l=generateTableString(sig)
    tableString=tableString +"double " +inputList[i] +" ("+ str(l) +",2)\n"
    tableString=tableString + s
    
#%% write to file

f=open("C:/Users/guser/TUCloud/MyTUSpace/SIC!/05_Smart Data/Modelica Modelle/ProcessHeating/inputTable.txt",'w')
f.write(tableString)
f.close()



