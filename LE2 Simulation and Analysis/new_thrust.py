import rocketcea
import numpy
import scipy
import matplotlib.pyplot as plt
from rocketcea.cea_obj import CEA_Obj
import matplotlib
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

#xetremely basic code that doesn't use three reservoir problem
chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
output = []
h = 0.001
time = 10
n = int(10/0.001)
time_array = numpy.linspace(0,10,n)
LOX_tank_volume = 0.00757

ETH_tank_volume = 0.006926

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
    mdot_LOX = Cd_LOX[0]*numpy.sqrt(2*rho_lox*(P_inj_LOX - P_chamber))
    mdot_Eth = Cd_Eth[0]*numpy.sqrt(2*rho_eth*(P_inj_Eth - P_chamber))



    #diffeq part for lox
    pressure_ox = pressure_ox + h*(-pressure_ox)*mdot_LOX*1.67 /(rho_lox*(LOX_tank_volume - LOX_volume))
    LOX_volume = LOX_volume - h*mdot_LOX/rho_lox


    #diffeq part for eth
    pressure_eth = pressure_eth + h*(-pressure_eth)*mdot_Eth*1.67 /(rho_eth*(ETH_tank_volume - ETH_volume))
    ETH_volume = ETH_volume + h*mdot_Eth/rho_eth
    P_chamber = (pressure_ox + pressure_eth)/6

    #get thrust
    pressure_ox = pressure_ox /6894.76
    pressure_eth = pressure_eth /6894.76
    P_chamber = P_chamber/6894.76
    thrust = chamber.estimate_Ambient_Isp(P_chamber,mdot_LOX/mdot_Eth,eps=4.36)[0]
    thrust = 10*thrust*(mdot_Eth+mdot_LOX)
    output.append(thrust)
    mdot.append(mdot_Eth+mdot_LOX)
    
#print(output)
#plt.plot(time_array,output)
print(mdot[0])
thrust = chamber.estimate_Ambient_Isp(325,0.451/0.282,eps=4.36)[0]
thrust = 10*thrust*(0.451+0.282)
print(thrust)
plt.plot(time_array,output)
#print(output[0])
#print([output[0],output[1000],output[2000],output[3000],output[4000],output[5000],output[6000],output[7000],output[8000],output[9000]])
plt.show()













 



