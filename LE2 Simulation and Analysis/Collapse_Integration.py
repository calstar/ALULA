import numpy as np
import math
import CoolProp.CoolProp as CP

#tank dims
V_oxtank = 6.92655*(10**-3) #m^3
A_oxtank = math.pi * (0.05715**2) #m^2
V_ethtank = 7.57*(10**-3) #m^3
A_ethtank = math.pi * (0.070612**2) #m^2


def Collapse_OX(P_oxtank_psi, V_oxinit = 3.25):

    P_oxtank = P_oxtank_psi * 6895 #convert to Pa

    #pressurant
    V_oxgas = V_oxtank - V_oxinit*(10**-3) #m^3
    MN2 = CP.PropsSI('M', 'Nitrogen') #kg/mol
    RN2_ox_molar = CP.PropsSI('GAS_CONSTANT', 'V', V_oxgas, 'T', 298.15, 'Nitrogen')/1000
    RN2_ox = RN2_ox_molar/MN2
    mN2_ox = (P_oxtank/1000 * V_oxgas) / (RN2_ox * 298.15)
    CpN2_ox = CP.PropsSI('CPMASS', 'P', P_oxtank, 'T', 298.15, 'Nitrogen')/1000

    # Diffeq conditions
    T0 = 90.2
    Ta = 300
    tSpan = np.linspace(0, 15, 2000) #to change time range edit this
    h = tSpan[1] - tSpan[0]
    T_new = np.zeros((len(tSpan), 2))
    T_new[0, :] = [T0, Ta]
    delta_T = [Ta - T0]

    # Constants from Ring
    C = 0.27
    n = 1/4

    CpLOX = CP.PropsSI('CPMASS', 'P', P_oxtank, 'T', 90.2, 'Oxygen')/1000
    CvLOX = CP.PropsSI('CVMASS', 'P', P_oxtank, 'T', 90.2, 'Oxygen')/1000
    muLOX = CP.PropsSI('VISCOSITY', 'P', P_oxtank, 'T', 90.2, 'Oxygen')/1000
    kfLOX = CP.PropsSI('CONDUCTIVITY', 'P', P_oxtank, 'T', 90.2, 'Oxygen')/1000
    rhoLOX = CP.PropsSI('DMASS', 'P', P_oxtank, 'T', 90.2, 'Oxygen')/1000
    betaLOX = 1 / CP.PropsSI('T', 'P', P_oxtank, 'Q', 0, 'Oxygen')  # coefficient of volume expansion
    mOX = rhoLOX * V_oxinit #kg
    LsLOX = V_oxinit/A_oxtank #characteristic length

    # Prandtl
    PrLOX = CpLOX * muLOX / kfLOX
    # Grashof
    GrLOX = (LsLOX**3) * rhoLOX * betaLOX * abs(delta_T[0]) / (muLOX**2)
    # Heat transfer coeff
    hLOX = [C * kfLOX * ((GrLOX * PrLOX)**n) / LsLOX]

    # Differential equation
    def convecLOX(X, h_LOX):
        alpha = (-mOX * CvLOX) / (mN2_ox * CpN2_ox)
        return np.array([alpha * h_LOX * (X[0] - X[1]), -h_LOX * (X[1] - X[0])])

    # RK4 integration
    for j in range(len(tSpan) - 1):
        k1 = convecLOX(T_new[j, :], hLOX[j])
        k2 = convecLOX(T_new[j, :] + 0.5 * h * k1, hLOX[j])
        k3 = convecLOX(T_new[j, :] + 0.5 * h * k2, hLOX[j])
        k4 = convecLOX(T_new[j, :] + h * k3, hLOX[j])
        T_new[j+1, :] = T_new[j, :] + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        delta_T.append(T_new[j+1, 1] - T_new[j+1, 0])
        GrLOX = (LsLOX**3) * rhoLOX * betaLOX * abs(delta_T[-1]) / (muLOX**2)
        hLOX.append(C * kfLOX * ((GrLOX * PrLOX)**n) / LsLOX)

    T_final = T_new[-1, 1]

    # Using Ideal gas law to get final pressure
    P_final = P_oxtank * (T_final / Ta)
    return P_final / 6895

def Collapse_ETH (P_ethtank_psi, V_ethinit = 3.00):

    P_ethtank = P_ethtank_psi * 6895 #convert to Pa

    #pressurant
    V_oxgas = V_oxtank - V_ethinit*(10**-3) #m^3
    MN2 = CP.PropsSI('M', 'Nitrogen') #kg/mol
    RN2_eth_molar = CP.PropsSI('GAS_CONSTANT', 'V', V_oxgas, 'T', 298.15, 'Nitrogen')/1000
    RN2_eth = RN2_eth_molar/MN2
    mN2_eth = (P_ethtank/1000 * V_oxgas) / (RN2_eth * 298.15)
    CpN2_eth = CP.PropsSI('CPMASS', 'P', P_ethtank, 'T', 298.15, 'Nitrogen')/1000

    # Diffeq conditions
    T0 = 293
    Ta = 300
    tSpan = np.linspace(0, 15, 2000) #to change time range edit this
    h = tSpan[1] - tSpan[0]
    T_new = np.zeros((len(tSpan), 2))
    T_new[0, :] = [T0, Ta]
    delta_T = [Ta - T0]

    # Constants from Ring
    C = 0.27
    n = 1/4

    CpETH = CP.PropsSI('CPMASS', 'P', P_ethtank, 'T', 298.15, 'Ethanol')/1000
    CvETH = CP.PropsSI('CVMASS', 'P', P_ethtank, 'T', 298.15, 'Ethanol')/1000
    muETH = CP.PropsSI('VISCOSITY', 'P', P_ethtank, 'T', 298.15, 'Ethanol')/1000
    kfETH = CP.PropsSI('CONDUCTIVITY', 'P', P_ethtank, 'T', 298.15, 'Ethanol')/1000
    rhoETH = CP.PropsSI('DMASS', 'P', P_ethtank, 'T', 298.15, 'Ethanol')/1000
    betaETH = 1 / CP.PropsSI('T', 'P', P_ethtank, 'Q', 0, 'Ethanol')
    mETH = rhoETH * V_ethinit
    LsETH = V_ethinit/A_ethtank

    # Prandtl
    PrETH = CpETH * muETH / kfETH
    # Grashof
    GrETH = (LsETH**3) * rhoETH * betaETH * abs(delta_T[0]) / (muETH**2)
    # Heat transfer coeff
    hETH = [C * kfETH* ((GrETH * PrETH)**n) / LsETH]

    # Differential equation
    def convecETH(X, h_ETH):
        alpha = (-mETH * CvETH) / (mN2_eth * CpN2_eth)
        return np.array([alpha * h_ETH * (X[0] - X[1]), -h_ETH * (X[1] - X[0])])

    # RK4 integration
    for j in range(len(tSpan) - 1):
        k1 = convecETH(T_new[j, :], hETH[j])
        k2 = convecETH(T_new[j, :] + 0.5 * h * k1, hETH[j])
        k3 = convecETH(T_new[j, :] + 0.5 * h * k2, hETH[j])
        k4 = convecETH(T_new[j, :] + h * k3, hETH[j])
        T_new[j+1, :] = T_new[j, :] + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        delta_T.append(T_new[j+1, 1] - T_new[j+1, 0])
        GrLOX = (LsETH**3) * rhoETH * betaETH * abs(delta_T[-1]) / (muETH**2)
        hETH.append(C * kfETH * ((GrLOX * PrETH)**n) / LsETH)

    T_final = T_new[-1, 1]

    # Using Ideal gas law to get final pressure
    P_final = P_ethtank * (T_final / Ta)
    return P_final / 6895

# Test
result1 = Collapse_OX(450)
print(f"Final Pressure LOX tank: {result1:.2f} psi")

result2 = Collapse_ETH(450)
print(f"Final Pressure eth tank: {result2:.2f} psi")



