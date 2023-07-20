import rocketcea
import os
import numpy as np
import scipy
from scipy.optimize import minimize
import matplotlib
matplotlib.use("TkAgg")
import tkinter as tk  
import matplotlib.pyplot as plt
print(matplotlib.matplotlib_fname())
from rocketcea.cea_obj import CEA_Obj
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
CdA_inj_LOX = 0.00001134 #faked for testing
CdA_inj_ETH = 0.00001078 #faked for testing


#Fluid Properties (SI units)
rho_LOX = 1140.0
rho_ETH = 798.0
gamma_tanks = 1.67 #edit this stuff for tank decompression calcs
V_oxtank = 6.92655 #L
V_ethtank = 7.57 #L
V_oxinit = 3.067 #optimize this!
V_ethinit = 3.078 #optimize this!
V_oxgas = V_oxtank-V_oxinit
V_ethgas = V_ethtank - V_ethinit

#Hydraulic Resistance Terms
R_ox = CdA_inj_LOX*math.sqrt((2*rho_LOX)) #mdot=R*(dP)^1/2
R_eth = CdA_inj_ETH*math.sqrt((2*rho_ETH)) #mdot=R*(dP)^1/2





#Shotgun Test - Initial Condition Only

#Initial Tank Pressures
P_tank_ox_psi = 475.0 #psia
P_oxtank = P_tank_ox_psi*6895 #Pa

P_tank_eth_psi = 475.0 #psia
P_ethtank = P_tank_eth_psi*6895 #Pa

#create Chamber Pressure Guess Array
Pc_test_psi = np.linspace(100.0, 450.0, 100) #psia
Pc_test = Pc_test_psi*6895 #Pa

#define cstar efficiency: completeion of energy release. See RPE Pg64
Efficiency = 0.92

#define Throat Diameter, Area
Dt = 26.04/1000 #m
At = Dt**2/4*math.pi

# Initialize Arrays
Residual = []

#solve residual for each pressure guess
for i in range(len(Pc_test)):
    #solve mass flow from fluid resistances
    mdot_ox = R_ox*((P_oxtank - Pc_test[i])**(1/2))
    mdot_eth = R_eth*((P_ethtank - Pc_test[i])**(1/2))
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


def Calculate_Residual(Pc, P_oxtank, P_ethtank):
    #solve mass flow from fluid resistances
    mdot_ox = R_ox*((P_oxtank - Pc)**(1/2))
    mdot_eth = R_eth*((P_ethtank - Pc)**(1/2))
    mdot_fluid = mdot_ox + mdot_eth
    #get OF ratio
    OF_ratio = mdot_ox/mdot_eth

    #solve mass flow from CEA 
    chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
    Cstar_fps = chamber.get_Cstar(Pc = Pc[0], MR=OF_ratio[0]) #see RPE pg64
    Cstar = Cstar_fps*0.3048 #m
    mdot_CEA = Pc*At/(Cstar*Efficiency) #kg/s

    #compare residual, append to array
    error = mdot_CEA-mdot_fluid
    
    global OF_ratio_glob
    OF_ratio_glob = OF_ratio

    global mdot_total_glob
    mdot_total_glob = (mdot_CEA+mdot_fluid)/2

    return error



def GradientDescent(guess, P_oxtank, P_ethtank):
    # Use scipy optimize minimize with residual function to find Chamber Pressure
    result = minimize(
        Calculate_Residual,
        guess,
        args = (P_oxtank, P_ethtank)
    )
    P_chamber = result.x[0]
    OF = OF_ratio_glob
    isp = chamber.estimate_Ambient_Isp(P_chamber,OF,eps=4.35)[0]
    thrust = 9.8*isp*(mdot_total_glob)
    print(f"Thrust {thrust}")
    massflow_total = mdot_total_glob
    print(f"PC {P_chamber/6895} and MR {OF_ratio} at {i*dt}")

    return P_chamber, thrust, OF, massflow_total



#reference thrust curve
time = np.linspace(0, 15, 20) #200 pts from 0 to 15 seconds
dt = time[2]-time[1]
OF_array = []
Thrust_array = []
P_chamber_array = []
fin = 0

for i in range(len(time)): #perform this for every timestep in the profile

    Pc_guess = 250*9800
    print(f"Oxtank = {P_oxtank/6895} Ethtank = {P_ethtank/6895} at {i*dt}")
    P_chamber, Thrust, OF, md_tot = GradientDescent(Pc_guess, P_oxtank, P_ethtank)
    md_ox = md_tot/(1+1/OF) 
    md_eth = md_tot-md_ox
    
    #print(f"Timestep {dt}")
    masslost_ox = md_ox*dt
    masslost_eth = md_eth*dt
    print(f"masslostox {masslost_ox}")
    
    print(f"Voxgas(L) {V_oxgas}")
    V_oxgas_next = V_oxgas + (masslost_ox/(rho_LOX*0.001))
    V_ethgas_next = V_ethgas + (masslost_eth/(rho_ETH*0.001))
    print(f"Voxgasnext(L) {V_oxgas_next}")

    P_oxtank = P_oxtank*(V_oxgas/V_oxgas_next)**gamma_tanks
    P_ethtank = P_ethtank*(V_ethgas/V_ethgas_next)**gamma_tanks

    V_oxgas = V_oxgas_next
    V_ethgas = V_oxgas_next

    
    OF_array.append(OF)
    Thrust_array.append(Thrust)
    P_chamber_array.append(P_chamber/6895)

    if V_oxgas[0]>V_oxtank or V_ethgas[0]>V_ethtank:
        print(type(V_oxgas))
        oxrem = V_oxtank-V_oxgas
        ethrem = V_ethtank-V_ethgas
        fin = 1
        print("finished")
        break
    if fin == 1:
        break

    
plt.figure()
plt.plot(time, Thrust_array)
plt.xlabel('Time(s)')
plt.ylabel('Thrust(N)')
plt.title('Thrust Curve')
plt.grid(True)
plt.show()

plt.figure()
plt.plot(time, P_chamber_array)
plt.xlabel('Time(s)')
plt.ylabel('Pressure')
plt.title('P_chamber Curve')
plt.grid(True)
plt.show()
