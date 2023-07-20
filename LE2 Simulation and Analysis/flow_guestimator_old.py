import rocketcea
import os
import numpy as np
import scipy
import matplotlib.pyplot as plt
from rocketcea.cea_obj import CEA_Obj
import matplotlib
import pandas as pd


def guesstimate(thrust , time): #takes an array of thrust vs time (say like 20 data points? idk. Uses the profile to "guess flowrate using CEA and Cds")
    n = len(time)
    chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
    OF = 0.9
    output = []
    '''change the lambda funcs below'''
    effective_res_ox = lambda P_c,P_ox : 0*P_c   #type in derived expression for calculating flowrate from an effective resistance and given chamber pressure
    effective_res_ox = lambda P_c,P_ox : 0*P_c
    for i in range(n):
        #perform this for every point on the profile
        objective = lambda P_c : chamber.estimate_Ambient_Isp(P_c,OF,eps=4.35)[0] - (effective_res_ox(P_c,pressure_ox) + effective_res_eth(P_c,pressure_eth))
        #ADD BISECTION METHOD CODE HERE


        output.append()

    return output