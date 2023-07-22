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

#Hydraulic Resistance Terms
R_ox = CdA_inj_LOX*math.sqrt((2*rho_LOX)) #mdot=R*(dP)^1/2
R_eth = CdA_inj_ETH*math.sqrt((2*rho_ETH)) #mdot=R*(dP)^1/2

#Tank Properties
gamma_tanks = 1.41 #1.41=GN2, 1.67=GHe
V_oxtank = 6.92655 #L
V_ethtank = 7.57 #L
V_oxinit = 3.65 #OPTMIMIZE THIS
V_ethinit = 3.75 #OPTIMIZE THIS
V_oxgas = V_oxtank-V_oxinit
V_ethgas = V_ethtank - V_ethinit


#Initial Tank Pressures
P_tank_ox_psi = 485.0 #psia
P_oxtank = P_tank_ox_psi*6895 #Pa

P_tank_eth_psi = 445.0 #psia
P_ethtank = P_tank_eth_psi*6895 #Pa

#define cstar efficiency: completeion of energy release. See RPE Pg64
Efficiency = 0.925
chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object

#define Throat Diameter, Area
Dt = 26.04/1000 #m
At = (Dt**2)/4*math.pi


#SHOTGUN TEST: INITIAL CONDITION

# Initialize Array
Residual = []

#create Chamber Pressure Guess Array
Pc_test_psi = np.linspace(100.0, 450.0, 100) #psia
Pc_test = Pc_test_psi*6895 #Pa

#solve residual for each pressure guess
for i in range(len(Pc_test)):
    
    #solve mass flow from fluid resistances
    mdot_ox = R_ox*((P_oxtank - Pc_test[i])**(1/2))
    mdot_eth = R_eth*((P_ethtank - Pc_test[i])**(1/2))
    mdot_fluid = mdot_ox + mdot_eth
    #get OF ratio
    OF_ratio = mdot_ox/mdot_eth

    #solve mass flow from CEA 
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
    mdot_ox = R_ox*((P_oxtank - (Pc*6895))**(1/2))
    mdot_eth = R_eth*((P_ethtank - (Pc*6895))**(1/2))
    mdot_fluid = mdot_ox + mdot_eth
    #get OF ratio
    OF_ratio = mdot_ox/mdot_eth
    #solve mass flow from CEA 
    Cstar_fps = chamber.get_Cstar(Pc = Pc[0], MR = OF_ratio[0]) #see RPE pg64
    Cstar = float(Cstar_fps*0.3048) #m/s
    if Cstar == 0:
        print("CSTAR ZERO")
        print(f"PC = {Pc[0]}")
        mdot_CEA_res = Pc*6895*At/(abs(Cstar)*Efficiency)
    else:
        mdot_CEA_res = (Pc*6895)*At/(Cstar*Efficiency) #kg/s

    #compare residual, append to array
    error = mdot_CEA_res-mdot_fluid
    
    global OF_ratio_glob
    OF_ratio_glob = float(OF_ratio[0])

    global mdot_total_glob
    mdot_total_glob = float(mdot_fluid[0])

    if OF_ratio > 1.8: #tank pressure drop eqs breaking
        error = 500

    return abs(error)



def GradientDescent(guess, P_oxtank, P_ethtank):
    # Use scipy optimize minimize with residual function to find Chamber Pressure
    result = minimize(
        Calculate_Residual,
        guess,
        args = (P_oxtank, P_ethtank),
        bounds = [(100, min([P_oxtank/6895, P_ethtank/6895]))],
    )
    P_chamber = result.x[0]
    # t = type(P_chamber)
    #print(f"Pchamb TYPE {t}")
    OF = OF_ratio_glob
    isp = chamber.estimate_Ambient_Isp(Pc=P_chamber,MR=OF,eps=4.35)[0]
    thrust = 9.8*isp*(mdot_total_glob)/1000 #kN
    #print(f"Thrust {thrust}")
    massflow_total = mdot_total_glob
    print(f"massflow {massflow_total}")
    #print(f"PC {P_chamber/6895} and MR {OF_ratio} at {i*dt}")

    return P_chamber, thrust, OF, massflow_total



#reference thrust curve
time = np.linspace(0, 20, 150) #200 pts from 0 to 15 seconds
dt = float(time[1]-time[0])
print(f"TIMESTEP {dt}")
OF_array = []
Thrust_array = []
P_chamber_array = []
mdtot_array = []
fin = 0

for i in range(len(time)): #perform this for every timestep in the profile

    if i == 0:
        Pc_guess = 350
    else:
        Pc_guess = P_chamber_last-5
    
    P_chamber, Thrust, OF, md_tot = GradientDescent(Pc_guess, P_oxtank, P_ethtank)
    md_ox = md_tot/(
        1+1/OF) 
    md_eth = md_tot-md_ox
    
    #print(f"Timestep {dt}")
    masslost_ox = md_ox*dt
    masslost_eth = md_eth*dt
    #print(f"masslostox {masslost_ox}")
    
    #print(f"Voxgas(L) {V_oxgas}")
    V_oxgas_next = V_oxgas + (masslost_ox/(rho_LOX*0.001))
    V_ethgas_next = V_ethgas + (masslost_eth/(rho_ETH*0.001))
    #print(f"Voxgasnext(L) {V_oxgas_next}")

    P_oxtank = P_oxtank*((V_oxgas/V_oxgas_next)**gamma_tanks)
    P_ethtank = P_ethtank*((V_ethgas/V_ethgas_next)**gamma_tanks)
    print(f"Oxtank = {P_oxtank/6895}[psi] ... Ethtank = {P_ethtank/6895}[psi] at {i*dt}")

    V_oxgas = V_oxgas_next
    V_ethgas = V_oxgas_next

    mdtot_array.append(md_tot)
    OF_array.append(OF)
    Thrust_array.append(Thrust)
    P_chamber_array.append(P_chamber)
    assert len(Thrust_array) == len(P_chamber_array)
    P_chamber_last = P_chamber

    if P_chamber>(0.90*P_oxtank/6895) or P_chamber>(0.90*P_oxtank/6895):
        OxDrop = (P_oxtank/6895)/P_chamber
        ETHDrop = (P_ethtank/6895)/P_chamber
        print(f"Flow Stability Violated with {OxDrop}% LOXratio and {ETHDrop}% ETHratio")
        break

    if V_oxgas>=(V_oxtank-V_oxtank/250) or V_ethgas>=(V_ethtank-V_ethtank/250):
        oxrem = V_oxtank-V_oxgas
        ethrem = V_ethtank-V_ethgas
        print(f"Burn finished with {oxrem}L LOX and {ethrem}L ETH")
        break
    
fig, axs = plt.subplots(2, 2)


# Add a scatter plot to the first subplot
sc = axs[0, 0].plot(time[0:len(Thrust_array)], Thrust_array)
axs[0, 0].set_title("Thrust Curve")
axs[0, 0].set_xlabel("Time(s)")
axs[0, 0].set_ylabel("Thrust(kN)")
#axs[0, 0].set_ylim(0, max(Thrust_array))  # Set y-axis limits

axs[0, 1].plot(time[0:len(Thrust_array)], P_chamber_array)
axs[0, 1].set_title("Chamber Pressure")
axs[0, 1].set_xlabel("Time(s)")
axs[0, 1].set_ylabel("Pchamber (psi)")

axs[1, 0].plot(time[0:len(Thrust_array)], mdtot_array)
axs[1, 0].set_title("Total Mass Flow")
axs[1, 0].set_xlabel("Time(s)")
axs[1, 0].set_ylabel("Mass Flow (kg/s)")

axs[1, 1].plot(time[0:len(Thrust_array)], OF_array)
axs[1, 1].set_title("OF Ratio")
axs[1, 1].set_xlabel("Time(s)")
axs[1, 1].set_ylabel("OF Ratio")
axs[1, 1].set_ylim(min(OF_array), 2)  # Set y-axis limits

plt.show()
