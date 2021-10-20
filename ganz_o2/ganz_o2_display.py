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
# CALLBACK
#class GuiCallback:
class Callback:


    def __init__(self, cfg, sens, graph, txt ):
        self.cfg        = cfg
        self.sens       = sens
        self.graph      = graph
        self.txt        = txt


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


    def timer( self ):
        m                           = self.sens.read()
        #self.sens.meas.adc_raw      = m['adc_raw' ]
        #self.sens.meas.temp_digC    = m['t_digc'  ]
        #self.sens.meas.pres_hPa     = m['p_hpa'   ]
        self.sens.meas.adc_mV       = self.sens.raw_to_mV( self.sens.meas.adc_raw )
        self.sens.meas.ppm_sw       = self.sens.raw_to_ppm( self.sens.meas.adc_raw,
                                                            self.sens.meas.temp_digC,
                                                            self.sens.meas.pres_hPa )

        #rmse = np.std( self.graph.ydata['O2'] )
        rmse = self.sens.rmse( self.graph.ydata['GAS'] )

        self.graph.push( self.graph.xdata,                  datetime.now()              )
        self.graph.push( self.graph.ydata['GAS'       ],    self.sens.meas.ppm_sw       )
        self.graph.push( self.graph.ydata['GAS F1'    ],    0.0                         )
        self.graph.push( self.graph.ydata['ADC RAW'   ],    self.sens.meas.adc_raw      )
        self.graph.push( self.graph.ydata['ADC mV'    ],    self.sens.meas.adc_mV       )
        self.graph.push( self.graph.ydata['t DigC'    ],    self.sens.meas.temp_digC    )
        self.graph.push( self.graph.ydata['P hPa'     ],    self.sens.meas.pres_hPa     )

        self.graph.plot()

        self.txt['O2'    ].set_text( '{:#.2f} PPM'   .format( self.sens.meas.ppm_sw ) )
        self.txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( self.sens.meas.adc_mV ) )
        self.txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( self.sens.meas.temp_digC ) )
        self.txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( self.sens.meas.pres_hPa ) )
        self.txt['RMS'   ].set_text( '{:#4.2f}'      .format( rmse ) )


    def button( self, event, label ):

        if label == 'P HIGH':
            self.sens.trim_p1()
            print( 'TRIM 1: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
            self.cfg['SENSOR']['tg'    ] = str( self.sens.tg     )
            self.cfg['SENSOR']['offset'] = str( self.sens.offset )
            self.cfg['SENSOR']['p0_raw'] = str( self.sens.p0_raw )
            self.cfg['SENSOR']['p1_raw'] = str( self.sens.p1_raw )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile: self.cfg.write( configfile )

        elif label == 'P ZERO':
            self.sens.trim_p0()
            print( 'TRIM 0: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
            self.cfg['SENSOR']['tg'    ] = str( self.sens.tg     )
            self.cfg['SENSOR']['offset'] = str( self.sens.offset )
            self.cfg['SENSOR']['p0_raw'] = str( self.sens.p0_raw )
            self.cfg['SENSOR']['p1_raw'] = str( self.sens.p1_raw )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile: self.cfg.write( configfile )

        elif label == 'K TEMP':
            self.sens.trim_drift_temp( self.sens.meas.temp_digC,  self.sens.y )
            print( 'K TEMP: ', self.sens.ktemp.raw_0, self.sens.ktemp.digc_0, self.sens.ktemp.raw_1, self.sens.ktemp.digc_1 )
            self.cfg['SENSOR']['ktemp_raw_0'    ] = str( self.sens.ktemp.raw_0      )
            self.cfg['SENSOR']['ktemp_raw_1'    ] = str( self.sens.ktemp.raw_1      )
            self.cfg['SENSOR']['ktemp_digc_0'   ] = str( self.sens.ktemp.digc_0     )
            self.cfg['SENSOR']['ktemp_digc_1'   ] = str( self.sens.ktemp.digc_1     )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile: self.cfg.write( configfile )

        elif label == 'K PRES':
            m       = self.sens.read()
            #print( 'p_hpa: ', type(m['p_hpa']) )
            self.sens.trim_kpres( m['p_hpa'] )

        else:
            print( label )


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    cfg     = ConfigParser()
    cfg['DEFAULT']['filename']  = "ganz.ini"
    cfg.read( cfg['DEFAULT']['filename'] )

    #gcfg    = ConfigParser()
    #gcfg.read( "o2mb_graph.ini")


    title   =   cfg['SENSOR']['modbus_port'] + '@' +            \
                cfg['SENSOR']['modbus_baudrate'] + ' ADDR: ' +  \
                cfg['SENSOR']['modbus_address']

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )

    ###########################################################################
    # SENSOR
    sens    = Sensor( cfg['SENSOR'] )

    ###########################################################################
    # GRAPH

    o2_min      = int( cfg['GRAPH'  ]['o2_min'      ] )
    o2_max      = int( cfg['GRAPH'  ]['o2_max'      ] )
    adc_raw_min = int( cfg['GRAPH'  ]['adc_raw_min' ] )
    adc_raw_max = int( cfg['GRAPH'  ]['adc_raw_max' ] )
    t_digc_min  = int( cfg['GRAPH'  ]['t_digc_min'  ] )
    t_digc_max  = int( cfg['GRAPH'  ]['t_digc_max'  ] )
    p_hpa_min   = int( cfg['GRAPH'  ]['p_hpa_min'   ] )
    p_hpa_max   = int( cfg['GRAPH'  ]['p_hpa_max'   ] )
    adc_mv_min  = int( cfg['GRAPH'  ]['adc_mv_min'  ] )
    adc_mv_max  = int( cfg['GRAPH'  ]['adc_mv_max'  ] )

    param   = [
    #   name        ymin            ymax            color       linestyle   linewidth
    (   'GAS',      o2_min,         o2_max,         'blue',     'dashed',   1,      ),
    (   'GAS F1',   o2_min,         o2_max,         'royalblue','dashed',   1,      ),
    (   'ADC RAW',  adc_raw_min,    adc_raw_max,    'orange',   'solid',    1,      ),
    (   'ADC mV',   adc_mv_min,     adc_mv_max,     'orange',   'dotted',   1,      ),
    (   't DigC',   t_digc_min,     t_digc_max,     'red',      'dashed',   1,      ),
    (   'P hPa',    p_hpa_min,      p_hpa_max,      'green',    'dashdot',  1,      ), ]

    xlen    = cfg.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.60], axes_class=HostAxes )
    graph   = Graph( ax, xlen, param )
    graph.init_timestamp( graph.xdata )

    graph.ydata['GAS']  = [0.0 for a in range( len(graph.ydata['GAS']))]
    #print( graph.ydata['t DIGC'])

    ###########################################################################
    # GUI
    ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2.set(xticks=[], yticks=[])
    ax2.axis["left","right","top","bottom"].set_visible(False)

    btn_ph  = Button( plt.axes([0.3, 0.90, 0.20, 0.05]), 'P HIGH'    )
    btn_pz  = Button( plt.axes([0.3, 0.85, 0.20, 0.05]), 'P ZERO'    )

    btn_ph.on_clicked( lambda x: cbk.button(x, btn_ph.label.get_text()) )
    btn_pz.on_clicked( lambda x: cbk.button(x, btn_pz.label.get_text()) )

    btn_kt  = Button( plt.axes([0.75, 0.90, 0.20, 0.05]), 'K TEMP'    )
    btn_kp  = Button( plt.axes([0.75, 0.85, 0.20, 0.05]), 'K PRES'    )

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
                    ('P HIGH',  'gray',     ),  ]

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
        htxt[ key[ 0] ].set_text( '{:#.2f} PPM'   .format( cfg.getfloat(key[0], 'ppm') ) )
        ypos    += 0.175


    ###########################################################################
    # CALLBACK
    cbk     = Callback( cfg, sens, graph, htxt )

    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer )
    timer.start()

    plt.show()
