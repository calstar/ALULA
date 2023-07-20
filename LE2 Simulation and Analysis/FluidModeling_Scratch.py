
import os
import numpy as np
import scipy
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import math 

CdA_inj_LOX = 0.00001134
CdA_inj_ETH = 0.00001078

#fluid Properties SI units
rho_LOX = 1140
rho_ETH = 798

#hydraulic resistance terms
R_ox = CdA_inj_LOX*math.sqrt((2*rho_LOX)) #mdot=R*(dP)^1/2
R_ETH = CdA_inj_ETH*math.sqrt((2*rho_ETH)) #mdot=R*(dP)^1/2



