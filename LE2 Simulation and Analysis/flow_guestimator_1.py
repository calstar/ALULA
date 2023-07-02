import rocketcea
import os
import numpy as np
import scipy
import matplotlib.pyplot as plt
from rocketcea.cea_obj import CEA_Obj
import matplotlib
import pandas as pd
import math 

#data from new_thrust.py
rho_ox = 1141
rho_eth = 798
Cd_eth = [0.0000172046,0.000010777] #T-I, I-C
Cd_ox = [0.0000258069,0.0000113]
P_tank_ox = [346.93,
            346.93,
            346.93,
            346.93,
            346.93,
            346.93,
            335.3,
            321.62,
            308.77,
            296.94,
            289.71,
            282.74,
            276.07,
            269.65,
            263.48,
            254.71,
            243.83,
            236.36,
            229.26,
            222.58,
            216.2,
            207.92,
            201.01]
P_tank_eth = [422.23,
            422.06,
            397.65,
            376.08,
            355.84,
            337.49,
            321.29,
            307.5,
            295.36,
            287.68,
            280.47,
            274.38,
            268.39,
            262.33,
            257.1,
            249.09,
            239.81,
            233,
            227.43,
            221.49,
            216.04,
            211.31,
            205.93]
P_c_exp = [-2.34,
                -2.01,
                187.49,
                223.75,
                229.11,
                223.41,
                212.39,
                200.05,
                187.49,
                179.65,
                171.88,
                164.55,
                157.74,
                151.51,
                145.46,
                137.35,
                127.78,
                121.42,
                115.82,
                110.19,
                101.59,
                28.34,
                10.1]
P_c_exp = np.array(P_c_exp) + 4
P_tank_eth = np.array(P_tank_eth) - 27
P_tank_ox = np.array(P_tank_ox) 

#lumped resistance terms
Cd_ox_tot = Cd_ox[0]*Cd_ox[1]/(Cd_ox[0] + Cd_ox[1])
Cd_eth_tot = Cd_eth[0]*Cd_eth[1]/(Cd_eth[0] + Cd_eth[1])

#reference thrust curve
time = np.linspace(0, 10, 20) #20 pts from 0 to 10 seconds
thrust = np.linspace(5000, 200, 20)

#terminates either within a specified range of the zero (tol) or after a specified number of iterations (n)
def bisection(f, a, b, tol, n = 100):

    header = "{:<10s}{:<15s}{:<15s}{:<15s}{:<10s}".format('j', 'a', 'b', 'p', 'f(p)')
    print(header)
    print('-' * (len(header)-6))
    
    for j in range(n):
        p = (a + b) / 2
        if p - a < tol:
            return p
        
        print(f"{j:2d}  {a:12.8f}  {b:12.8f}  {p:12.8f}  {f(p):12.8f}")

        if f(a) * f(p) > 0:
            a = p
        else:
            b = p
    print("\nafter", n, "iterations, estimate is", p)
    return p


def guesstimate(thrust , time): #takes an array of thrust vs time (say like 20 data points? idk. Uses the profile to "guess flowrate using CEA and Cds")
    t = len(time)
    chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
    OF = 0.9 #ideal

    #lambda funcs
    mdot_ox = lambda P_c, P_tank_ox: Cd_ox_tot * ((2*rho_ox*(abs(P_c - P_tank_ox)))**(1/2))
    mdot_eth = lambda P_c, P_tank_eth: Cd_eth_tot * ((2*rho_eth*(abs(P_c - P_tank_eth)))**(1/2))

    mdot_2 = lambda P_c, P_tank_ox, P_tank_eth: mdot_ox(P_c, P_tank_ox) + mdot_eth(P_c, P_tank_eth)
    # mdot_1 = lambda thrust, P_c_exp, OF, eps=4.35: thrust/(chamber.estimate_Ambient_Isp(P_c_exp,OF,eps=4.35)[0]*9.8) #pasted in nonlambda form to avoid confusion

    output = [] #estimated chamber pressures
    objective_vals = [] #for plotting

    for i in range(t): #perform this for every point on the profile
        #fix the thrust, tank pressures, chamber pressure vals for each iteration
        thrust_i = thrust[i]
        P_tank_eth_i = P_tank_eth[i]
        P_tank_ox_i = P_tank_ox[i]
        P_c_exp_i = P_c_exp[i]
        objective = lambda P_c : thrust_i / ((chamber.estimate_Ambient_Isp(P_c_exp_i, OF, eps = 4.35)[0]) * 9.8) - mdot_2(P_c, P_tank_ox_i, P_tank_eth_i)
        root = bisection(objective, 0, 500, 1e-10, 1000)
        output.append(root)

        #for plotting
        obj_val = objective(root)
        objective_vals.append(obj_val)

    objective_vals = np.array(objective_vals)
    plt.figure()
    plt.plot(time, objective_vals)
    plt.xlabel('Time')
    plt.ylabel('Objective Function Value at an estimated root why is this so sus')
    plt.title('Objective Function Value vs Time')
    plt.grid(True)
    plt.show()

    print(output)
    return output

guesstimate(thrust , time)
#-------------------------


# def guesstimate(thrust , time): #takes an array of thrust vs time (say like 20 data points? idk. Uses the profile to "guess flowrate using CEA and Cds")
#     t = len(time)
#     chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
#     OF = 0.9 #ideal
#     output = []
#     '''change the lambda funcs below'''
#     effective_res_ox = lambda P_c,P_ox : 0*P_c   #type in derived expression for calculating flowrate from an effective resistance and given chamber pressure
#     effective_res_eth = lambda P_c,P_eth : 0*P_c
#     for i in range(t):
#         #perform this for every point on the profile
#         objective = lambda P_c : chamber.estimate_Ambient_Isp(P_c,OF,eps=4.35)[0] - (effective_res_ox(P_c,pressure_ox) + effective_res_eth(P_c,pressure_eth)) #fixed pt mdot - mdot!
#         #ADD BISECTION METHOD CODE HERE
#         bisection(objective, )
        


#         output.append()

#     return output