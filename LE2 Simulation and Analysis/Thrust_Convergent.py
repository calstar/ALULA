import rocketcea
import os
import numpy as np
import scipy
import matplotlib.pyplot as plt
from rocketcea.cea_obj import CEA_Obj
import matplotlib
import pandas as pd
import math 

class propellant:
    def __init__(self,name, density, viscosity,tank_length, tank_radius, temperature):
        self.density = density
        self.viscosity = viscosity
        self.temp = temperature
        self.rad = tank_radius
        self.length = tank_length
        self.name = name
        return None
    def __str__(self) -> str:
        return self.name


#System Test Data Results
CdA_inj_LOX = 0.00005
CdA_inj_ETH = 0.00004

#Fluid Properties (SI units)
rho_LOX = 1140
rho_ETH = 798

#Hydraulic Resistance Terms
R_LOX = CdA_inj_LOX*math.sqrt((2*rho_LOX)) #mdot=R*(dP)^1/2
R_ETH = CdA_inj_ETH*math.sqrt((2*rho_ETH)) #mdot=R*(dP)^1/2





#Shotgun Test - Initial Condition Only

#Initial Tank Pressures
P_tank_LOX_psi = 450 #psia
P_tank_LOX = P_tank_LOX_psi*6895 #Pa

P_tank_ETH_psi = 475 #psia
P_tank_ETH = P_tank_eth_psi*6895 #Pa

#create Chamber Pressure Guess Array
Pc_test_psi = np.linspace(100, 500, 50) #psia
Pc_test = Pc_test_psi*6895 #Pa

#define cstar efficiency: completeion of energy release. See RPE Pg64
Efficiency = 0.9

#define Throat Diameter, Area
Dt = 26.04/1000 #m
At = Dt^2/4*math.pi

# Initialize Arrays
Residual = []

#solve residual for each pressure guess
for i in range(len(Pc_test)):
    #solve mass flow from fluid resistances
    mdot_ox = R_LOX*math.sqrt(Pc_test[i]-P_tank_LOX)
    mdot_eth = R_ETH*math.sqrt(Pc_test[i]-P_tank_ETH)
    mdot_fluid = mdot_ox + mdot_eth
    #get OF ratio
    OF_ratio = mdot_ox/mdot_eth

    #solve mass flow from CEA 
    chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
    Cstar_fps = chamber.get_Cstar(Pc=Pc_test_psi[i], MR=OF_ratio) #see RPE pg64
    Cstar = Cstar_fps*0.3048 #m
    mdot_CEA = Pc_test[i]*At/(Cstar*Efficiency) #kg/s

    #compare residual, append to array
    error = mdot_CEA-mdot_fluid
    Residual.append(abs(error))


plt.figure()
plt.plot(Pc_test_psi, Residual)
plt.xlabel('Chamber Pressure Guess (psia)')
plt.ylabel('Residual Error (abs(kg/s))')
plt.title('Chamber Pressure - Residual Solution Space')
plt.grid(True)
plt.show()











#Bisection Method Time Curve

#reference thrust curve
time = np.linspace(0, 15, 200) #200 pts from 0 to 15 seconds
thrust = np.linspace(5000, 200, 200)

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

# guesstimate(thrust , time)
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