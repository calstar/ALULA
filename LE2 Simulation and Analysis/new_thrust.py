import rocketcea
import os
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

ox_pres = [346.93,346.93,
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
# ox_FR = np.array(data["FlowrateOx(Gal/s)"])
# eth_FR = np.array(data["FlowrateEth(Gal/s)"])

ox_pres = np.array(ox_pres) 

eth_pres = [422.23,
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
eth_pres = np.array(eth_pres) - 27
chamber_pres = [-2.34,
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
chamber_press = np.array(chamber_pres) + 4
#implement simple grad descent(using scipy.optimize for now) to learn the best linear function of tank pressures to get chamber pressure
#check for local minima when we add in velocities
f = lambda a: a[0]*ox_pres + a[1]*eth_pres 
loss = lambda a: (np.linalg.norm((f(a) - chamber_pres)))**2
a_star = scipy.optimize.minimize(loss,[0,0])
a_star = a_star.x
print(a_star)




#xetremely basic code that doesn't use three reservoir problem
#first elem is feed system, second is injector
#first Cd is feed system
#second is injector Cd


#for optimization problem fix MAx press and max tank volume
time = 3
h = 0.001
n = int(time/h)
def thrust(LOX_tank_volume = 0.00757,ETH_tank_volume = 0.006926,Cd_Eth = [0.0000172046,0.000010777] 
        ,Cd_LOX = [0.0000258069,0.0000113],pressure_ox = 100,pressure_eth = 100,P_chamber = 50): 
    chamber = CEA_Obj(propName="", oxName="LOX", fuelName="C2H5OH") #initializs CEA object
    output = []
    h = 0.001
    time = 3
    n = int(time/h)
    time_array = []
    

    

    G2M3 = 0.00378541
    f1 = 0.255*G2M3*798
    f3 = 0.023*G2M3*1141
    print(f1+f3)
    rho_lox = 1141
    rho_eth = 798
    psi_to_Pa = 6894.76
    ETH_volume = 2.5/rho_eth #in m^3
    LOX_volume = 3.5/rho_lox
    
    mdot = []
    pE = []
    pO = []
    mO = []
    mE = []
    for i in range(n):
        time_array.append(i*time/n)
        pressure_ox = pressure_ox*6894.76
        pressure_eth = pressure_eth*6894.76
        P_chamber = P_chamber*6894.76
        
        k_eth = (Cd_Eth[0]/Cd_Eth[1])*(Cd_Eth[0]/Cd_Eth[1])
        k_lox = (Cd_LOX[0]/Cd_LOX[1])*(Cd_LOX[0]/Cd_LOX[1])
        P_inj_LOX = (k_lox*pressure_ox + P_chamber)/(k_lox + 1)
        P_inj_Eth = (k_eth*pressure_eth + P_chamber)/(k_eth + 1)
        mdot_LOX = Cd_LOX[0]*np.sqrt(2*rho_lox*(P_inj_LOX - P_chamber)) 
        mdot_Eth = Cd_Eth[0]*np.sqrt(2*rho_eth*(P_inj_Eth - P_chamber))



        #diffeq part for lox
        pressure_ox = pressure_ox - h*(pressure_ox)*mdot_LOX /(rho_lox*(LOX_tank_volume - LOX_volume))
        LOX_volume = LOX_volume - h*mdot_LOX/rho_lox
        mO.append(LOX_volume*rho_lox)


        #diffeq part for eth
        pressure_eth = pressure_eth - h*(pressure_eth)*mdot_Eth /(rho_eth*(ETH_tank_volume - ETH_volume))
        ETH_volume = ETH_volume - h*mdot_Eth/rho_eth
        mE.append(ETH_volume*rho_eth)
        # P_chamber = a_star[0]*pressure_ox + a_star[1]*pressure_eth 

        #get thrust
        pressure_ox = pressure_ox /6894.76
        pressure_eth = pressure_eth /6894.76
        P_chamber = P_chamber/6894.76
        isp = chamber.estimate_Ambient_Isp(P_chamber,mdot_LOX/mdot_Eth,eps=4.35)[0]
        thrust = 9.8*thrust*(mdot_Eth+mdot_LOX)
        output.append(thrust)
        mdot.append(mdot_Eth+mdot_LOX)
        pE.append(pressure_eth)
        pO.append(pressure_ox)
        if mE[len(mE)-1] <= 0 or mO[len(mO)-1] <= 0:
            break
        
    #print(output)
    #plt.plot(time_array,output)
    # print(mdot[0])
    thrust = chamber.estimate_Ambient_Isp(325,f3/f1,eps=1)[0]
    thrust = 10*thrust*(f3+f1)
    print(thrust)
    print(a_star)
    output = np.array(output)
    pE = np.array(pE)
    pO = np.array(pO)
    fig, axs = plt.subplots(3,3)
    axs[0,0].plot(time_array,output)
    axs[0,0].set_title("thrust")
    axs[1,0].plot(time_array,pE)
    axs[1,0].set_title("Press_ETH")
    axs[2,0].plot(time_array,pO)
    axs[2,0].set_title("Press_LOX")
    axs[1,1].plot(time_array,mE)
    axs[1,1].set_title("mass_ETH")
    axs[1,2].plot(time_array,mO)
    axs[1,2].set_title("mass_LOX")
    sum = 0.5*h*mdot[0] + 0.5*h*mdot[len(mdot) - 1] 
    for i in mdot[1:len(mdot) - 1]:
        sum = sum + h*i
    print(sum)
    return output , time_array
    # plt.plot(np.linspace(0,5,(len(ox_pres))),ox_pres)
    # plt.plot(np.linspace(0,5,(len(eth_pres))),eth_pres)
    #print(output[0])
    #print([output[0],output[1000],output[2000],output[3000],output[4000],output[5000],output[6000],output[7000],output[8000],output[9000]])
thrust_arr , time_arr = thrust()
if not os.path.exists("LE2.eng"): #change filename to include the path that you want
    f = open("LE2.eng", "x")  #change file name here too
    f.close()
    f = open("LE2.eng", "w")
else:
    f = open("LE2.eng","w") #and here
f.write("; Rocketvision F32" + "\n") #add header file info here. I've added the website default, change it to whatever you want
f.write("; from NAR data sheet updated 11/2000" + "\n")
f.write("; created by John Coker 5/2006")
f.write("F32 24 124 5-10-15 .0377 .0695 RV")
f.write("\n")  #add a \n whenever you need to move to the next line
for i in range(n):
    f.write("%s" + "%s" + "%s" + time_arr[i] + "%s" + thrust_arr[i] + "\n") 
plt.show()



