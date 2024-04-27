import numpy as np
import scipy.optimize as opt
import matplotlib.pyplot as plt

# Let's start by transcribing the formula from the image into a Python function.

def calculate_nusselt_number(Re_D, Pr):
    """
    Calculate the Nusselt number using the Churchill and Bernstein equation.

    Parameters:
    Re_D (float): Reynolds number based on diameter.
    Pr (float): Prandtl number.

    Returns:
    float: Nusselt number.
    """
    Nu_D = (0.3 + 
            (0.62 * Re_D**0.5 * Pr**(1/3)) / 
            ((1 + (0.4 / Pr)**(2/3))**(1/4)) *
            (1 + (Re_D / 282000)**(5/8))**(4/5)
           )
    return Nu_D


def Rconv(T, v, d, L): #units in MKS(m kg s K)
    #thermal resistance of cylinder in forced crossflow. Constant temperatures
    Re = 4*v*d/1.5e-5
    Pr = 0.7
    k = 0.0257
    Nu = calculate_nusselt_number(Re, Pr)
    h = d*Nu/(k)
    A = np.pi*d*L
    Rconv = 1/h*A
    return Rconv

def Rcond(ro, ri, k, L):
    #thermal resistance of conductive cylinder insulation. Constant internal temp
    Rcond = np.log(ro/ri)/(2*np.pi*k*L)
    return Rcond

k_insulation = 0.05
L = 1 #length of cylinder in meters
v = 10 #windspeed in m/s
T = 21 #wind temperature in C
T = 21+273 #wind temperature in K

ri = 4.5 * 2.54 / 100 # Convert to meters
r0 = 6.75 * 2.54 / 100 # Convert to meters

v_values = np.linspace(1, 15, 100) # Windspeed values in m/s
Rtotal_values = []

for v in v_values:
    Rtotal = Rconv(T, v, r0, L) + Rcond(r0, ri, k_insulation, L)
    Rtotal_values.append(Rtotal)

plt.plot(v_values, Rtotal_values)
plt.xlabel('Windspeed (m/s)')
plt.ylabel('Rtotal (K/W)')
plt.title('Total Thermal Resistance vs Windspeed')
plt.grid(True)
plt.show()

ri = 0.5 * 2.54 / 100 # Convert to meters
r0_values = np.linspace(0.55, 3, 100)
r0_values = r0_values * 2.54 / 100 # Convert to meters
Rtotal_values = []

for r0 in r0_values:
    Rtotal = Rconv(300, 10, r0, L) + Rcond(r0, ri, k_insulation, L)
    Rtotal_values.append(Rtotal)

plt.plot((r0_values-ri)*100, Rtotal_values)
plt.xlabel('th (cm)')
plt.ylabel('Rtotal (K/W)')
plt.title('Total Thermal Resistance vs r0')
plt.grid(True)
plt.show()