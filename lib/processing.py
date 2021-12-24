import numpy as np
import math
import sys
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
# SENSOR STORAGE
class   SensorMeasure:
    __slots__       =   'adc_raw', 'adc_mV', 'ppm_sw', 'ppm_hw',            \
                        'temp_digc', 'pres_hpa',                            \
                        'mcu_digc', 'mcu_vdda',                             \
                        'temp_raw', 'pres_raw', 'slope_raw', 'offset_raw',

class   SensorStatus:
    __slots__       =   'error_code', 'starts_cnt',                         \
                        'adc_vref', 'adc_res_bits',                         \
                        'raw2ppm_fv', 'raw2ppm_ft', 'raw2ppm_fp',

class   SensorKtemp:
    __slots__       =   'k', 'raw_0', 'raw_1', 'digc_0', 'digc_1',

class   SensorKpres:
    __slots__       =   'k', 'raw_0', 'raw_1', 'hpa_0', 'hpa_1',

class   SensorTrim:
    __slots__       =    'slope', 'offset', 'raw', 'ppm',


###############################################################################
# SENSOR
class Processing:

    def __init__(self, coef):

        #__slots__           = 'y', 'p',

        #self.coef           = [float()] * idx
        #self.coef           = [0.0] * idx
        self.coef           = coef


    def coef_init(self, c):
        pass


###############################################################################
# CALLBACK
#class GuiCallback:
class Callback:


    def __init__(self, cfg, sens ):
        self.cfg        = cfg
        self.sens       = sens


    def trim_0(self, event):
        self.sens.trim_p0()
        print( 'TRIM 0: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
        #self.cfg['SENSOR']['tg'    ] = str( self.sens.tg     )
        #self.cfg['SENSOR']['offset'] = str( self.sens.offset )
        self.cfg['SENSOR']['p0_raw'] = str( self.sens.p0_raw )
        self.cfg['SENSOR']['p1_raw'] = str( self.sens.p1_raw )
        with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
            self.cfg.write( configfile )


    def trim_1(self, event):
        self.sens.trim_p1()
        print( 'TRIM 1: ', self.sens.p0_ppm, self.sens.p0_raw, self.sens.p1_ppm, self.sens.p1_raw )
        #self.cfg['SENSOR']['tg'    ] = str( self.sens.tg     )
        #self.cfg['SENSOR']['offset'] = str( self.sens.offset )
        self.cfg['SENSOR']['p0_raw'] = str( self.sens.p0_raw )
        self.cfg['SENSOR']['p1_raw'] = str( self.sens.p1_raw )
        with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
            self.cfg.write( configfile )

    '''
    def timer_cb(self, sens, graph, txt ):

        m   = sens.read()
        ppm = sens.raw_to_ppm( m['adc_raw'], m['t_digc'], m['p_hpa'] )

        graph.push( graph.buf['timestamp'], datetime.now()  )
        graph.push( graph.buf['adc_raw'  ], m['adc_raw' ]   )
        graph.push( graph.buf['t_digc'   ], m['t_digc'  ]   )
        graph.push( graph.buf['p_hpa'    ], m['p_hpa'   ]   )
        graph.push( graph.buf['ppm'      ], ppm             )

        txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( sens.raw_to_mV(m['adc_raw' ])  ) )
        txt['PPM'   ].set_text( '{:#.2f} PPM'   .format( ppm                            ) )
        txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( m['t_digc' ]                   ) )
        txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( m['p_hpa' ]                    ) )

        #graph.plot()
        ybufs   = [ graph.buf['ppm'], graph.buf['adc_raw'], graph.buf['t_digc'], graph.buf['p_hpa'] ]
        graph.plot( graph.buf['timestamp'], ybufs )
    '''

    def timer( self, txt ):
        meas, sts = self.sens.read()
        ppm     = self.sens.raw_to_ppm( meas.adc_raw, meas.temp_digc, meas.pres_hpa )
        adc_mV  = self.sens.raw_to_mV( meas.adc_raw )

        txt['O2'    ].set_text( '{:#.2f} PPM'   .format( ppm                            ) )
        txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( sens.raw_to_mV(meas.adc_raw)   ) )
        txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( meas.temp_digc                 ) )
        txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( meas.pres_hpa                  ) )
        #txt['RMS'   ].set_text( '{:#4.2f}'      .format( rmse ) )
        #txt['SLOPE' ].set_text( '{:#4.2f}'    .format( sd ) )
        #txt['P HIGH'].set_text( '{:#4.2f}'      .format( rmse ) )

        print( "| %8.2f | %3.2f | %4.2f | %8d |" %
                (meas.ppm_hw, meas.temp_digc, meas.pres_hpa, meas.slope_raw ), end = '\r' )


    def button( self, event, lbl ):
        print( lbl )


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()
    conf['DEFAULT']['config_path']  = "ganz.ini"
    conf.read( conf['DEFAULT']['config_path'] )

    title   =   conf['MODBUS']['port'] + '@' +            \
                conf['MODBUS']['baudrate'] + ' ADDR: ' +  \
                conf['MODBUS']['address']

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )

    ###########################################################################
    # SENSOR
    sens    = Sensor( conf )

    ###########################################################################
    # GRAPH
    '''
    param   = [
    #   name        ymin    ymax        color       linestyle   linewidth
    (   'O2',       -50,    150,        'blue',     'dashed',   1,      ),
    (   'ADC RAW',  0,      10000000,   'orange',   'dashed',   1,      ),
    (   'ADC mV',   0,      2500,       'orange',   'dotted',   1,      ),
    (   't DIGC',   15,     45,         'red',      'dashdot',  1,      ),
    (   'P hPa',    950,    1050,       'green',    'dotted',   1,      ), ]

    xlen    = conf.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.60], axes_class=HostAxes )
    graph   = Graph( ax, xlen, param )
    graph.init_timestamp( graph.xdata )

    graph.ydata['O2']   = [0.0 for a in range( len(graph.ydata['O2']))]
    #print( graph.ydata['t DIGC'])
    '''

    ###########################################################################
    # CALLBACK
    cbk     = Callback( conf, sens )

    ###########################################################################
    # GUI
    ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2.set(xticks=[], yticks=[])
    ax2.axis["left","right","top","bottom"].set_visible(False)

    hbtn_pz  = Button( plt.axes([0.3, 0.70, 0.20, 0.05]), 'P ZERO'    )
    hbtn_p1  = Button( plt.axes([0.3, 0.75, 0.20, 0.05]), 'P 1'       )
    hbtn_p2  = Button( plt.axes([0.3, 0.80, 0.20, 0.05]), 'P 2'       )
    hbtn_p3  = Button( plt.axes([0.3, 0.85, 0.20, 0.05]), 'P 3'       )
    hbtn_ph  = Button( plt.axes([0.3, 0.90, 0.20, 0.05]), 'P HIGH'    )

    hbtn_pz.on_clicked( lambda x: cbk.button(x, hbtn_pz.label.get_text()) )
    hbtn_p1.on_clicked( lambda x: cbk.button(x, hbtn_p1.label.get_text()) )
    hbtn_p2.on_clicked( lambda x: cbk.button(x, hbtn_p2.label.get_text()) )
    hbtn_p3.on_clicked( lambda x: cbk.button(x, hbtn_p3.label.get_text()) )
    hbtn_ph.on_clicked( lambda x: cbk.button(x, hbtn_ph.label.get_text()) )

    txt_left    = [ ('O2',      'blue',     ),
                    ('ADC',     'orange',   ),
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
