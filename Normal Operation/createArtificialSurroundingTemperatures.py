# -*- coding: utf-8 -*-
"""
Created on Tue Jun 30 14:51:32 2020

@author: Gernot Steindl



"""
import math as m
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

#
def generateArtificialTempValues(colName,timeRange, meanValue, amp, frequency, shift, distortionStd):
    """ Generates a cosine function with some random distortion.
    
    Parameters
    ----------
    colName : String
        Name of the column in the created pandas data frame.
    timeRange : data_range
        time stamped index.
    meanValue : double
        Offset of the cosine.
    amp : double
        Amplitude of the cosine function.
    frequency : double
        DESCRIPTION.
    shift : double
        DESCRIPTION.
    distortionStd : double
        The distortion is modelled as normal distribution with zero mean. This parameter sets the standard deviation.

    Returns
    -------
    DataFrame
        A time series data frame.

    """
    length=timeRange.size
    index=np.arange(0,length)
    distortion=np.random.normal(0,distortionStd,length)
    
    temp=np.cos(index*2*m.pi*frequency+shift*2*m.pi)*amp +distortion + meanValue
    
    return pd.DataFrame(temp, index=timeIndex, columns=[colName])
    

#%% generate 2 random signals for 3 days in a row
timeIndex=pd.date_range(start='2020-04-15', end="2020-04-17 23:00", freq='h')

ambTemp=generateArtificialTempValues('T_in',timeIndex,10+273.15,3,1/24,5/24,0.5)
indoorTemp=generateArtificialTempValues('T_surr',timeIndex,20+273.15,1,1/24,5/24,0.2)

ambTempMin=ambTemp.resample('min').interpolate(method='cubic')
indoorTempMin=indoorTemp.resample('min').interpolate(method='cubic')

#ambTempMin.plot()
(indoorTempMin-273).plot()


#%% write data frames to modlica input table format (Modelica.Blocks.Sources.CombiTimeTable)
FILE_PATH="./inputSurroundingTemperatures.txt"
inputList=(ambTempMin,indoorTempMin)

#add version string - as required 
tableString="#1\n"

for data in inputList:

    dataLength=len(data.index)
    numIndex=index=np.arange(0,dataLength)

    #add header string
    tableString=tableString +"double " + data.columns[0] +" ("+ str(dataLength) +",2)\n"
    
    #create table string: time - value pair
    table=""
    for i in range(dataLength):
        tableString=tableString + str(numIndex[i])+"\t"+str(data.values[i][0]) + "\n"
    #add 
    
   # tableString=tableString + s
        
f=open(FILE_PATH,'w')
f.write(tableString)
f.close()