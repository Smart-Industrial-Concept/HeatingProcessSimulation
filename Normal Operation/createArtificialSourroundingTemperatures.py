# -*- coding: utf-8 -*-
"""
Created on Tue Jun 30 14:51:32 2020

@author: guser
"""
import math as m
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


timeIndex=pd.date_range(start='2020-04-15', end="2020-04-17 23:00", freq='h')

length=timeIndex.size
t_h=np.arange(0,length)
np.random.seed(4)


distortion=np.random.normal(0,0.5,length)
mean=10

temp=np.cos(t_h*2*m.pi/24+5/24*2*m.pi)*3 +distortion + mean
ambTemp=pd.DataFrame(temp, index=timeIndex)

ambTempMin=ambTemp.resample('min').interpolate(method='cubic')

ambTempMin.plot()

#%%
mean=20
shift_h=5
distortion=np.random.normal(0,0.2,length)

temp=np.cos(t_h*2*m.pi/24+shift_h/24*2*m.pi)*1 +distortion + mean

indoorTemp=pd.DataFrame(temp, index=timeIndex)
indoorTemp.plot()