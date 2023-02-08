import rocketcea
import numpy
import scipy
import matplotlib.pyplot as plt
from rocketcea.cea_obj import CEA_Obj
class Numerical_stuff:#contains things like RK4 and stuff
    def __init__(self):
        pass
    def RK4(f,alpha,a,b,N):
        #again, just does what the function says, Likely to replace with Backward-Differentiation-Formula-3 for stability concerns
        h = (b-a) / N
        t = [a + i*h for i in range(N)]
        w = [alpha]
        for i in range(1,N):
            k1 = h*f(w[i])
            k2 = h*f(w[i] + k1/2)
            k3 = h*f(w[i] + k2/2)
            k4 = h*f(w[i] + k3)
            w.append(w[i] + (k1 + 2*k2 + 2*k3 + k4)/6)
        return w
    def RK4ns(f,alpha,a,b,N):
         h = (b-a) / N
         k1 = h*f(alpha)
         k2 = h*f(alpha + k1/2)
         k3 = h*f(alpha + k2/2)
         k4 = h*f(alpha + k3)
         return alpha +  (k1 + 2*k2 + 2*k3 + k4)/6
    def integrate(arr,time,h):
        N = time/h
        sum = 0
        for i in range(N):
            if i == 0 or i == N - 1:
                sum = sum + h*arr[i]
            else :
                sum = sum + 2*h*arr[i]
        return sum



    def derivative(arr,index,time):
        #literally does what the function name says, O(h) for end points, h^2 elsewhere
        if index == 0:
            return (arr[1] - arr[0])/(time[1] - time[0])
        elif index == len(arr) - 1 :
            return (arr[index] - arr[index - 1])/(time[index] - time[index - 1])
        else :
            return (arr[index+1] - arr[index - 1])/(time[index+1] - time[index - 1])
    def secant_method(f,u1,u2,tol):
        x = [u1,u2]
        while abs(x[len(x) - 1] - x[len(x) - 2]) < tol:
            x.append(x[len(x) - 2]*f(x[len(x) - 1]) - x[len(x) - 1]*f(x[len(x) - 2]))
            x[len(x) - 1] = x[len(x) - 1]/(f(x[len(x) - 1]) - f(x[len(x) - 2]))
        return x[len(x) - 1]

#just makes things nicer to read. takes in name of propellant, density and viscosity in that order
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
    def get_Reynolds(self,diam,mdot):
        A = numpy.pi*(diam/2)^2
        return mdot*diam/(A*self.viscosity)


 #adding equations like Darcy_weisbach, Navier-Stokes, Smagorinsky,etc to this class
class flow_functions:
    def CW(l,Re,eps,diameter):
        #calculates Colebrook-White friction factor
        a = 2.51/Re             #Re is Reynolds number
        b = 4*eps/(14.8*diameter)       #eps is friction factor of pipe
        inter = (numpy.log(10)/(2*a))*(numpy.power(10,b/(2*a)))
        Z = 1/numpy.power((2*scipy.special.lambertw(inter)/numpy.log(10) - b/a),2)      #W is the lambert W function which is the inverse of xe^x
        return Z
    def CW_wmdot(l,mdot,eps,diameter,fluid) :
        Re = fluid.get_Reynolds(diameter,mdot)
        return flow_functions.CW(l,Re,eps,diameter)
    #derivative of Colebrook_White wrt Reynolds number

    def dCW(l,Re,eps,diameter):
        a = 2.51/Re             #Re is Reynolds number
        b = 4*eps/(14.8*diameter)       #eps is friction factor of pipe
        X = a*numpy.power(numpy.log(10),2)
        Y = 2*(scipy.special.lambertw(numpy.power(10,b/a)*numpy.log(10)/a) + 1)
        Z = (a*(scipy.special.lambertw(numpy.power(10,b/a)*numpy.log(10)/a)) - b*numpy.log10)^3
        return (X/(Y*Z))*(-2.51/(Re*Re))

    def dP_bend(Re,diameter,eps,density,velocity,dff):   #dff is the Darcy Friction Factor
        Re_star = (2.89/(1+1000*eps/diameter))^12
        if 2320<=Re and Re<2*10^5 :
            if eps/diameter>=0.001 :
                K = (128*dff*0.21)*numpy.sqrt(diameter/0.036322) + ((numpy.pi/2*0.036322/diameter)*dff)
            elif eps/diameter < 0.001 :
                if Re>= Re_star :
                    K = (64*dff*0.21*(1+1000*eps/diameter))*numpy.sqrt(diameter/0.036322)+(numpy.pi/2*0.036322/diameter*dff)
                elif Re< Re_star :
                    K = 58.8*numpy.sqrt(diameter/0.036322)*0.21/(Re^(1/3))+(numpy.pi/2*0.036322/diameter*dff)
            elif eps/diameter == 0 :
                K = 58.8*numpy.sqrt(diameter/0.036322)*0.21/(Re^(1/3))+(numpy.pi/2*0.036322/diameter*dff)
        elif Re>=2*10^5 :
            if eps/diameter>=0.001 :
                K = 0.42*numpy.sqrt(diameter/0.036322)+(numpy.pi/2*0.036322/diameter*dff)
            elif eps/diameter < 0.001 :
                K = 0.21*(1+1000*eps/diameter)*numpy.sqrt(diameter/0.036322)+(numpy.pi/2*0.036322/diameter*dff)
            elif eps/diameter == 0 :
                K = 0.21*numpy.sqrt(diameter/0.036322)+(numpy.pi/2*0.036322/diameter*dff)
        return (K*density/2*velocity^2)


def get_thrust(fuel,oxidizer,total_time,pipes,fittings,pipes_fuel,fittings_fuel):
    #this section just initializes all params
    integrate = Numerical_stuff.integrate
    chamber = CEA_Obj(propName="", oxName=oxidizer.name, fuelName=fuel.name) #initializs CEA object
    temp = {}
    N = 1000
    h = total_time/N
    T = [i*h for i in range(N)]
    eps = {}
    pipe_length = {}
    cylinder_length = {fuel.name : fuel.length,
                    oxidizer.name : oxidizer.length}
    cylinder_area = {fuel.name : numpy.pi*fuel.rad*fuel.rad,
                    oxidizer.name : numpy.pi*oxidizer.rad*oxidizer.rad}
    K = {}
    R = {}
    chamber_temp = []
    tank_pressure_ox = []
    tank_pressure_fuel = []
    mdot_ox = []
    mdot_fuel = []
    chamber_pressure = []
    thrust = []

    #set initial values
    chamber_temp[0] = 300 #whatever ambient temperature is in Kelvin
    tank_pressure_ox[0] = 3103000 #450 psi in Pascal, could possibly update this using Sara's collapse factor just to mkae everything more streamlined
    tank_pressure_fuel[0] = 3103000
    chamber_pressure[0] = 103125 #1 atm in Pa, or whatever ambient temperature is
    area_exit_ox = 1
    area_exit_fuel = 1
    mdot_ox[0] = 0.99*area_exit_ox*numpy.sqrt(2*tank_pressure_ox[0]*oxidizer.density + 20*oxidizer.mass*oxidizer.density/cylinder_area[oxidizer.name])
    mdot_fuel[0] = 0.94*area_exit_fuel*numpy.sqrt(2*tank_pressure_fuel[0]*fuel.density + 20*fuel.mass*fuel.density/cylinder_area[fuel.name])
    new_pressure_ox = tank_pressure_ox[0] + oxidizer.mass/oxidizer.area
    new_pressure_fuel = tank_pressure_fuel[0] + fuel.mass/fuel.area
    #move through pipes and fittings


   #start the solve. Refer to Latex documentation for explanation of model

    for t in T:
        tank_pressure_ox.append((-5/3)*mdot_ox[t]/(oxidizer.volume*oxidizer.density - integrate(mdot_ox,t,h)) + tank_pressure_ox[t])
        tank_pressure_fuel.append((-5/3)*mdot_fuel[t]/(fuel.volume*fuel.density - integrate(mdot_ox,t,h)) + tank_pressure_fuel[t])
        mdot_ox[t] = 0.99*area_exit_ox*numpy.sqrt(2*tank_pressure_ox[0]*oxidizer.density + 20*oxidizer.mass*oxidizer.density/cylinder_area[oxidizer.name])
        mdot_fuel[t] = 0.94*area_exit_fuel*numpy.sqrt(2*tank_pressure_fuel[0]*fuel.density + 20*fuel.mass*fuel.density/cylinder_area[fuel.name])
        for i in len(pipes):
            new_pressure_ox = -0.5*pipes[i].length*flow_functions.CW(pipes[i].length,oxidizer.get_Reynolds(pipes[i].diam,mdot_ox[0]))*mdot_ox*mdot_ox/(oxidizer.density*pipes[i].area*pipes[i].area*pipes[i].diam) + new_pressure_ox
            new_pressure_ox = new_pressure_ox - mdot_ox*mdot_ox/(2*fittings[i].Cd*fittings[i].Cd*pipes[i].area*pipes[i].area*oxidizer.density)
        for i in len(pipes_fuel):
            new_pressure_fuel = -0.5*pipes_fuel[i].length*flow_functions.CW(pipes_fuel[i].length,fuel.get_Reynolds(pipes_fuel[i].diam,mdot_fuel[0]))*mdot_fuel*mdot_fuel/(fuel.density*pipes_fuel[i].area*pipes_fuel[i].area*pipes_fuel[i].diam) + new_pressure_fuel
            new_pressure_fuel = new_pressure_fuel - mdot_fuel*mdot_fuel/(2*fittings_fuel[i].Cd*fittings_fuel[i].Cd*pipes_fuel[i].area*pipes_fuel[i].area*fuel.density)



 #Carolline's part to update chamber pressure

        for i in len(massfracD.keys()):
            mole_i = massfracD[i][1] * (mdot_fuel[t] + mdot_ox[t]) * 1 / (N*moleWtD[i][0] )
            if massfracD[i] == "CO":
                #Ppc = sum(mole_i*Pci)
                #Tpc = sum(mole_i*Tci)
                #Ppr = P/Ppc
                #Tpr = T/Tpc
                #Given Tpr and Ppr, can find Z
                #This gives Z for an entire gas mixture (?)



                compressibility[i] =
            elif massfracD[i] == "CO2":
                compressibility[i] = 0.9
            elif massfracD[i] == "#H2":
                compressibility[i] =
            elif massfracD[i] == "H2O":
                compressibility[i] =
            elif massfracD[i] == "*OH":
                compressibility[i] =
            elif massfracD[i] == "*O2":
                compressibility[i] =
            else :
                compressibility[i] = 1

        # CO CO2 H HO2 H2 H2O H2O2 O OH O2
        # *CO *CO2 *H HO2 *H2 H2O H2O2 *O *OH *O2 1000psia, MR6    100psia, MR6
        # *CO *CO2 (COOH) *H (HCO) HO2 *H2 H2O H2O2 *O *OH *O2 1000psia, MR2


            partialp_i = compressibility[i] * mole_i * R * temperatures[x] / volume_chamber
            totalp += partialp_i
            pressures.append(totalp)
#now that we have chamber pressure and flow rate, can call get_thrust from CEA

    return thrust






















#combustion part
