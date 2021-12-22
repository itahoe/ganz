import sys
sys.path.append('../lib')

import numpy as np
import math
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
# CALLBACK
#class GuiCallback:
class Callback:


    def __init__(self, cfg, sens, graph):
        self.cfg        = cfg
        self.sens       = sens
        self.graph      = graph


    def trim_0(self, event):
        self.sens.trim_p0()
        print( 'TRIM 0: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
        self.cfg['SENSOR']['tg'    ] = str( self.sens.tg     )
        self.cfg['SENSOR']['offset'] = str( self.sens.offset )
        self.cfg['SENSOR']['p0_raw'] = str( self.sens.p0_raw )
        self.cfg['SENSOR']['p1_raw'] = str( self.sens.p1_raw )
        with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
            self.cfg.write( configfile )


    def trim_1(self, event):
        self.sens.trim_p1()
        print( 'TRIM 1: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
        self.cfg['SENSOR']['tg'    ] = str( self.sens.tg     )
        self.cfg['SENSOR']['offset'] = str( self.sens.offset )
        self.cfg['SENSOR']['p0_raw'] = str( self.sens.p0_raw )
        self.cfg['SENSOR']['p1_raw'] = str( self.sens.p1_raw )
        with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
            self.cfg.write( configfile )


    def timer( self, txt ):
        self.sens.read()
        self.sens.meas.adc_mV       = self.sens.meas.adc_raw  * (3300/(2**12))
        self.sens.meas.ppm_sw       = self.sens.raw_to_ppm( self.sens.meas.adc_raw,
                                                            self.sens.meas.temp_digc,
                                                            self.sens.meas.pres_hpa )

        average = 0
        xsum    = 0
        for x in self.graph.ydata['O2']:   average += x
        average /= len(self.graph.ydata['O2'])

        for x in self.graph.ydata['O2']:   xsum += (x - average)**2
        xsum    /= ( len(self.graph.ydata['O2']) - 1)
        rmse    = math.sqrt( xsum )


        adc_avrg = 0
        for x in self.graph.ydata['ADC mV']:   adc_avrg += x
        adc_avrg /= len(self.graph.ydata['ADC mV'])

        #self.sens.meas.adc_avrg

        self.graph.push( self.graph.xdata,                  datetime.now()  )
        self.graph.push( self.graph.ydata['O2'        ],    self.sens.meas.ppm_sw       )
        self.graph.push( self.graph.ydata['ADC mV'    ],    self.sens.meas.adc_mV       )
        self.graph.push( self.graph.ydata['Avrg mV'   ],    adc_avrg                    )
        self.graph.push( self.graph.ydata['t DIGC'    ],    self.sens.meas.temp_digc    )
        self.graph.push( self.graph.ydata['P hPa'     ],    self.sens.meas.pres_hpa     )

        self.graph.plot()

        txt['O2'        ].set_text( '{:#.2f} PPM'   .format( self.sens.meas.ppm_sw          ) )
        txt['ADC'       ].set_text( '{:#4.2f} mV'   .format( self.sens.meas.adc_raw         ) )
        txt['AVERAGE'   ].set_text( '{:#4.2f} mV'   .format( adc_avrg                       ) )
        txt['TEMP'      ].set_text( '{:#4.2f} °C'   .format( self.sens.meas.temp_digc       ) )
        txt['PRES'      ].set_text( '{:#4.2f} hPa'  .format( self.sens.meas.pres_hpa        ) )
        txt['RMS'       ].set_text( '{:#4.2f}'      .format( rmse ) )
        #txt['SLOPE' ].set_text( '{:#4.2f}'    .format( sd ) )
        #txt['P HIGH'].set_text( '{:#4.2f}'      .format( rmse ) )


    def button( self, event, lbl ):
        print( lbl )


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()
    conf['DEFAULT']['filename']  = "ganz.ini"
    conf.read( conf['DEFAULT']['filename'] )

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
    param   = [
    #   name        ymin    ymax        color       linestyle   linewidth
    (   'O2',       -50,    150,        'blue',     'dashed',   1,      ),
    (   'ADC mV',   0,      3300,       'orange',   'dotted',   3,      ),
    (   'Avrg mV',  0,      3300,       'gray',     'dotted',   3,      ),
    (   't DIGC',   15,     45,         'red',      'dashdot',  1,      ),
    (   'P hPa',    950,    1050,       'green',    'dotted',   1,      ), ]

    xlen    = conf.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.60], axes_class=HostAxes )
    graph   = Graph( ax, xlen, param )
    graph.init_timestamp( graph.xdata )

    graph.ydata['O2']       = [0.0 for a in range( len(graph.ydata['O2']))]
    graph.ydata['ADC mV']   = [0.0 for a in range( len(graph.ydata['ADC mV']))]

    ###########################################################################
    # CALLBACK
    cbk     = Callback( conf, sens, graph )

    ###########################################################################
    # GUI
    ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2.set(xticks=[], yticks=[])
    ax2.axis["left","right","top","bottom"].set_visible(False)

    btn_ph  = Button( plt.axes([0.3, 0.90, 0.20, 0.05]), 'P HIGH'    )
    btn_p3  = Button( plt.axes([0.3, 0.85, 0.20, 0.05]), 'P 3'       )
    btn_p2  = Button( plt.axes([0.3, 0.80, 0.20, 0.05]), 'P 2'       )
    btn_p1  = Button( plt.axes([0.3, 0.75, 0.20, 0.05]), 'P 1'       )
    btn_pz  = Button( plt.axes([0.3, 0.70, 0.20, 0.05]), 'P ZERO'    )

    btn_ph.on_clicked( lambda x: cbk.button(x, btn_ph.label.get_text()) )
    btn_p3.on_clicked( lambda x: cbk.button(x, btn_p3.label.get_text()) )
    btn_p2.on_clicked( lambda x: cbk.button(x, btn_p2.label.get_text()) )
    btn_p1.on_clicked( lambda x: cbk.button(x, btn_p1.label.get_text()) )
    btn_pz.on_clicked( lambda x: cbk.button(x, btn_pz.label.get_text()) )

    txt_left    = [ ('O2',      'blue',     ),
                    ('ADC',     'orange',   ),
                    ('AVERAGE', 'gray',     ),
                    ('TEMP',    'red',      ),
                    ('PRES',    'green',    ),
                    ('RMS',     'blue',     ),
                    ('SLOPE',   'gray',     ),  ]

    txt_trim    = [ ('P ZERO',  'gray',     ),
                    ('P 1',     'gray',     ),
                    ('P 2',     'gray',     ),
                    ('P 3',     'gray',     ),
                    ('P HIGH',  'gray',     ),  ]

    htxt        = {}

    xpos, ypos = 0.05, 0.90
    for key in txt_left:
        ax2.text( xpos, ypos, key[ 0]+': ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.1

    xpos, ypos = 0.60, 0.225
    for key in txt_trim:
        #ax2.text( xpos, ypos, key[0]+': ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ].set_color( key[ 1] )
        htxt[ key[ 0] ].set_text( '{:#.2f} PPM'   .format( conf.getfloat(key[0], 'ppm') ) )
        ypos    += 0.175


    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer, htxt )
    timer.start()

    plt.show()
