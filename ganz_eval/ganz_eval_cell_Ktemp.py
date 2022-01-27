import sys
sys.path.append('../lib')

import numpy as np
from scipy.signal   import kaiserord, butter, lfilter, lfilter_zi, filtfilt, firwin, freqz
from graph          import Graph
from sensor         import Sensor
from threading      import Timer
import matplotlib.pyplot as plt
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
import serial
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
import modbus_tk
import time, threading
from datetime import datetime, timedelta
from matplotlib.widgets import Button
from configparser import ConfigParser



def Ktemp_ITG():
    pass

def Ktemp_Al( t_degc, ppm ):
    t_kelvin    = t_degc + 273.15
    percent     = ppm * 10000
    return( (percent * (-0.17 / t_kelvin)) / 10000 )


###############################################################################
# MAIN
if __name__ == '__main__':


    ###########################################################################
    # FIGURE
    fig     = plt.figure()
    fig.canvas.manager.set_window_title( 'Cell Ktemp Evaluation' )

    ###########################################################################
    # GRAPH
    ppm_min = int( 0 )
    ppm_max = int( 1000 )

    t = np.arange(0., 100., 1)


    #plt.plot(ppm, ppm, 'r--', ppm, ppm**2, 'bs', ppm, ppm**3, 'g^')
    #plt.plot(param)
    plt.plot( t,    t,      'r--',   )
    plt.plot( t,    Ktemp_Al( t, 1000 ),   'b.',     )
    plt.plot( t,    t+10,   'g-',     )

    plt.grid(True)

    ###########################################################################
    # COLOR MAP
    #fig.patch.set_facecolor('#202020')
    #fig.patch.set_facecolor('black')
    fig.patch.set_facecolor('#F8F8F8')


    plt.show()
