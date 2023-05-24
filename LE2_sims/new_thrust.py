import rocketcea
import numpy as np
import scipy
import matplotlib.pyplot as plt
from rocketcea.cea_obj import CEA_Obj
import matplotlib
import pandas as pd
#6330 to 6354
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





#adjust thrust calculation using hotfire attempt
# fO = lambda x : x >= 164
# fE = lambda x : x >= 191
# fC = lambda x : x >= 100
# data = pd.read_excel(r"hotfire2_2023-04-16.xlsx")

# exp_thrust = np.array(data["total LCs"])
# ox_FR = np.array(data["FlowrateOx(Gal/s)"])
# eth_FR = np.array(data["FlowrateEth(Gal/s)"])
# ox_pres = (data["PT_01 (psi)"])
# ox_pres = np.array(ox_pres) 
# # ox_pres = list(filter(fO,ox_pres))
# print(ox_pres)
# eth_pres = list(filter(fE,(data["PT_E1 (psi)"])))
# eth_pres = np.array(eth_pres) - min(eth_pres)
# chamber_pres = list(filter(fC,(data["PT_C1 (psi)"])))
# chamber_press = np.array(chamber_pres) - min(chamber_pres)
# #implement simple grad descent(using scipy.optimize for now) to learn the best linear function of tank pressures to get chamber pressure

# f = lambda a: a[0]*ox_pres + a[1]*eth_pres + a[2]
# loss = lambda a: (np.linalg.norm((f(a) - chamber_pres)))**2
# a_star = scipy.optimize.minimize(loss,[0,0,0])
a_star = [1/6,1/6,1/3]




#xetremely basic code that doesn't use three reservoir problem
chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
output = []
h = 0.001
time = 10
n = int(10/0.001)
time_array = np.linspace(0,10,n)
LOX_tank_volume = 0.00757

ETH_tank_volume = 0.006926

G2M3 = 0.00378541
f1 = 0.255*G2M3*798
f3 = 0.023*G2M3*1141
rho_lox = 1141
rho_eth = 798
psi_to_Pa = 6894.76
ETH_volume = 2.75/rho_eth #in m^3
LOX_volume = 3.575/rho_lox
pressure_ox = 444.7
pressure_eth = 411.4
P_chamber = 325
mdot = []
for i in range(n):
    pressure_ox = pressure_ox*6894.76
    pressure_eth = pressure_eth*6894.76
    P_chamber = P_chamber*6894.76
    Cd_Eth = [0.0000172046,0.000010777] #first elem is feed system, second is injector
    Cd_LOX = [0.0000258069,0.0000113] 
    k_eth = (Cd_Eth[0]/Cd_Eth[1])*(Cd_Eth[0]/Cd_Eth[1])
    k_lox = (Cd_LOX[0]/Cd_LOX[1])*(Cd_LOX[0]/Cd_LOX[1])
    P_inj_LOX = (k_lox*pressure_ox + P_chamber)/(k_lox + 1)
    P_inj_Eth = (k_eth*pressure_eth + P_chamber)/(k_eth + 1)
    mdot_LOX = Cd_LOX[0]*np.sqrt(2*rho_lox*(P_inj_LOX - P_chamber))
    mdot_Eth = Cd_Eth[0]*np.sqrt(2*rho_eth*(P_inj_Eth - P_chamber))



    #diffeq part for lox
    pressure_ox = pressure_ox + h*(-pressure_ox)*mdot_LOX*1.5 /(rho_lox*(LOX_tank_volume - LOX_volume))
    LOX_volume = LOX_volume - h*mdot_LOX/rho_lox


    #diffeq part for eth
    pressure_eth = pressure_eth + h*(-pressure_eth)*mdot_Eth*1.4 /(rho_eth*(ETH_tank_volume - ETH_volume))
    ETH_volume = ETH_volume + h*mdot_Eth/rho_eth
    P_chamber = a_star[0]*pressure_ox + a_star[1]*pressure_eth + a_star[2]

    #get thrust
    pressure_ox = pressure_ox /6894.76
    pressure_eth = pressure_eth /6894.76
    P_chamber = P_chamber/6894.76
    thrust = chamber.estimate_Ambient_Isp(P_chamber,mdot_LOX/mdot_Eth,eps=4.35)[0]
    thrust = 10*thrust*(mdot_Eth+mdot_LOX)/2.7
    output.append(thrust)
    mdot.append(mdot_Eth+mdot_LOX)
    
#print(output)
#plt.plot(time_array,output)
# print(mdot[0])
thrust = chamber.estimate_Ambient_Isp(325,f3/f1,eps=1)[0]
thrust = 10*thrust*(f3+f1)
print(thrust)
plt.plot(time_array,output)
# plt.plot(np.linspace(0,5,(len(ox_pres))),ox_pres)
# plt.plot(np.linspace(0,5,(len(eth_pres))),eth_pres)
#print(output[0])
#print([output[0],output[1000],output[2000],output[3000],output[4000],output[5000],output[6000],output[7000],output[8000],output[9000]])
plt.show()













 



