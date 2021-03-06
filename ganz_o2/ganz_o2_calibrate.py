'''

a + b*x + c*x^2 + d*x^3

x = 10
100             = 1.1*10^3 + x*10^2 + 3.3*10 + 0
100 - 0         = 1.1*10^3 + x*10^2 + 3.3*10
100             = 1.1*10^3 + x*10^2 + 3.3*10
100 - 3.3*10    = 1.1*10^3 + x*10^2
67              = 1.1*10^3 + x*10^2
67 - 1.1*10^3   = x*10^2
-1033           = x*10^2
-1033 / 10^2    = x
-10.33          = x

'''


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




###############################################################################
# RAW2PPM
def raw2ppm( poly, raw ):
    ppm = poly.a + (poly.b * raw) + (poly.c * raw**2) + (poly.d * raw**3)
    return ppm


def poly_cal_a( poly, ppm, raw ):
    poly.a = ppm - (poly.b * raw) - (poly.c * raw**2) - (poly.d * raw**3)


def poly_cal_b( poly, ppm, raw ):
    poly.b = (ppm - poly.a - (poly.c * raw**2) - (poly.d * raw**3)) / raw


def poly_cal_c( poly, ppm, raw ):
    poly.c = (ppm - poly.a - (poly.b * raw) - (poly.d * raw**3)) / raw**2


def poly_cal( coef, idx, ppm, raw ):

    if idx < len(coef):
        s = 0.0
        for i in range(len(coef)):
            if i==idx:
                pass
            else:
                s += coef[i] * raw**i

        coef[idx] = (ppm - s) / raw**idx
    else:
        #raise ValueError("idx value > len(coef)")
        pass

    return coef


###############################################################################
# CALLBACK
class Callback:

    def __init__(self, conf, sens, graph, txt):
        self.conf       = conf
        self.sens       = sens
        self.graph      = graph
        self.txt        = txt

    '''
    def trim_0(self, event):
        self.sens.trim_p0()
        self.sens.trim_save( self.conf['DEFAULT']['filename'] )
        print( 'TRIM 0: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )


    def trim_1(self, event):
        self.sens.trim_p1()
        self.sens.trim_save( self.conf['DEFAULT']['filename'] )
        print( 'TRIM 1: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
    '''

    def timer( self ):
        self.sens.read()
        self.sens.meas.adc_mV   = self.sens.raw_to_mV( self.sens.meas.adc_raw )
        self.sens.meas.ppm_sw   = self.sens.raw_to_ppm( self.sens.meas.adc_raw,
                                                            self.sens.meas.temp_digc,
                                                            self.sens.meas.pres_hpa )

        #rmse = np.std( self.graph.ydata['O2'] )
        rmse = self.sens.rmse( self.graph.ydata['GAS'] )

        self.graph.push( self.graph.xdata,                  datetime.now()              )
        self.graph.push( self.graph.ydata['GAS'       ],    self.sens.meas.ppm_sw       )
        self.graph.push( self.graph.ydata['ADC RAW'   ],    self.sens.meas.adc_raw      )
        self.graph.push( self.graph.ydata['ADC mV'    ],    self.sens.meas.adc_mV       )
        self.graph.push( self.graph.ydata['t DigC'    ],    self.sens.meas.temp_digc    )
        self.graph.push( self.graph.ydata['P hPa'     ],    self.sens.meas.pres_hpa     )

        self.graph.plot()

        self.txt['O2'    ].set_text( '{:#.2f} PPM'   .format( self.sens.meas.ppm_sw )   )
        self.txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( self.sens.meas.adc_mV )   )
        self.txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( self.sens.meas.temp_digc ))
        self.txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( self.sens.meas.pres_hpa ) )
        self.txt['RMS'   ].set_text( '{:#4.2f}'      .format( rmse )                    )


    def button( self, event, label ):
        if label == 'P SPAN':
            self.sens.trim_p1()
            self.conf[label]['ppm'] = str( self.sens.trim.ppm[ 1] )
            self.conf[label]['raw'] = str( self.sens.trim.raw[ 1] )
            self.conf_save()
        elif label == 'P ZERO':
            self.sens.trim_p0()
            self.conf[label]['ppm'] = str( self.sens.trim.ppm[ 0] )
            self.conf[label]['raw'] = str( self.sens.trim.raw[ 0] )
            self.conf_save()
        elif label == 'K TEMP':
            self.sens.trim_drift_temp( self.sens.meas.temp_digc,  self.sens.y[-1] )
        elif label == 'K PRES':
            self.sens.trim_kpres( self.sens.meas.pres_hpa )
        else:
            print( label )


    def conf_save( self ):
        confpath   = self.conf['DEFAULT']['config_path']
        with open( confpath, "w" ) as configfile:
            self.conf.write( configfile )


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()

    conf.read( conf['DEFAULT']['config_path']
    print( conf['DEFAULT']['config_path'] )

    ###########################################################################
    # FIGURE
    title   =   conf['MODBUS']['port'    ] + '@' + \
                conf['MODBUS']['baudrate'] + ' ADDR: ' +  \
                conf['MODBUS']['address' ]

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )

    ###########################################################################
    # SENSOR
    sens    = Sensor( conf )

    ###########################################################################
    # GRAPH

    o2_min      = int( conf['GRAPH'  ]['o2_min'      ] )
    o2_max      = int( conf['GRAPH'  ]['o2_max'      ] )
    adc_raw_min = int( conf['GRAPH'  ]['adc_raw_min' ] )
    adc_raw_max = int( conf['GRAPH'  ]['adc_raw_max' ] )
    t_digc_min  = int( conf['GRAPH'  ]['t_digc_min'  ] )
    t_digc_max  = int( conf['GRAPH'  ]['t_digc_max'  ] )
    p_hpa_min   = int( conf['GRAPH'  ]['p_hpa_min'   ] )
    p_hpa_max   = int( conf['GRAPH'  ]['p_hpa_max'   ] )
    adc_mv_min  = int( conf['GRAPH'  ]['adc_mv_min'  ] )
    adc_mv_max  = int( conf['GRAPH'  ]['adc_mv_max'  ] )

    param   = [
    #   name        ymin            ymax            color       linestyle   linewidth
    (   'GAS',      o2_min,         o2_max,         'blue',     'dashed',   1,      ),
    (   'ADC RAW',  adc_raw_min,    adc_raw_max,    'orange',   'solid',    1,      ),
    (   'ADC mV',   adc_mv_min,     adc_mv_max,     'orange',   'dotted',   1,      ),
    (   't DigC',   t_digc_min,     t_digc_max,     'red',      'dashed',   1,      ),
    (   'P hPa',    p_hpa_min,      p_hpa_max,      'green',    'dashdot',  1,      ), ]

    xlen    = conf.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.25], axes_class=HostAxes )
    graph   = Graph( ax, xlen, param )
    graph.init_timestamp( graph.xdata )

    graph.ydata['GAS']  = [0.0 for a in range( len(graph.ydata['GAS']))]
    #print( graph.ydata['t DIGC'])

    ###########################################################################
    # GUI
    ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2.set(xticks=[], yticks=[])
    ax2.axis["left","right","top","bottom"].set_visible(False)

    btn_ph  = Button( plt.axes([0.30, 0.90, 0.20, 0.05]), 'P SPAN'    )
    btn_pz  = Button( plt.axes([0.30, 0.70, 0.20, 0.05]), 'P ZERO'    )

    btn_ph.on_clicked( lambda x: cbk.button(x, btn_ph.label.get_text()) )
    btn_pz.on_clicked( lambda x: cbk.button(x, btn_pz.label.get_text()) )

    btn_kt  = Button( plt.axes([0.30, 0.60, 0.20, 0.05]), 'K TEMP'    )
    btn_kp  = Button( plt.axes([0.30, 0.55, 0.20, 0.05]), 'K PRES'    )

    btn_kt.on_clicked( lambda x: cbk.button(x, btn_kt.label.get_text()) )
    btn_kp.on_clicked( lambda x: cbk.button(x, btn_kp.label.get_text()) )

    txt_left    = [ ('O2',      'blue',     ),
                    ('ADC',     'orange',   ),
                    ('TEMP',    'red',      ),
                    ('PRES',    'green',    ),
                    #('RMS',     'blue',     ),
                    ('RMS',     'yellow',   ),
                    ('SLOPE',   'gray',     ),  ]

    txt_trim    = [ ('P ZERO',  'gray',     ),
                    ('P SPAN',  'gray',     ),  ]

    htxt        = {}

    xpos, ypos = 0.05, 0.90
    for key in txt_left:
        ax2.text( xpos, ypos, key[ 0]+':  ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.1

    htxt['RMS'].set_backgroundcolor('gray')


    #xpos, ypos = 0.60, 0.225
    xpos, ypos = 0.60, 0.75
    for key in txt_trim:
        #ax2.text( xpos, ypos, key[0]+': ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ].set_color( key[ 1] )
        htxt[ key[ 0] ].set_text( '{:#.2f} PPM'   .format( conf.getfloat(key[0], 'ppm') ) )
        ypos    += 0.175


    ###########################################################################
    # CALLBACK
    cbk     = Callback( conf, sens, graph, htxt )

    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer )
    timer.start()

    plt.show()
