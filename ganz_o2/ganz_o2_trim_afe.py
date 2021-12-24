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
class Callback:

    def __init__(self, conf, sens, graph, txt):
        self.conf       = conf
        self.sens       = sens
        self.graph      = graph
        self.txt        = txt

    def timer( self ):
        self.sens.read_measure()

        adc_mV  = self.sens.raw_to_mV( self.sens.meas.adc_raw )
        ppm_sw  = self.sens.raw_to_ppm( self.sens.meas.adc_raw,
                                        self.sens.meas.temp_digc,
                                        self.sens.meas.pres_hpa )

        #rmse = np.std( self.graph.ydata['O2'] )
        rmse = self.sens.rmse( self.graph.ydata['GAS PPM'] )

        self.graph.push( self.graph.xdata,                  datetime.now()              )
        self.graph.push( self.graph.ydata['GAS PPM'     ],  ppm_sw                      )
        #self.graph.push( self.graph.ydata['ADC RAW'     ],    self.sens.meas.adc_raw      )
        self.graph.push( self.graph.ydata['ADC mV'      ],  adc_mV                      )
        self.graph.push( self.graph.ydata['t DigC'      ],  self.sens.meas.temp_digc    )
        self.graph.push( self.graph.ydata['P hPa'       ],  self.sens.meas.pres_hpa     )

        self.graph.plot()

        self.txt['GAS SW'   ].set_text( '{:#.2f} PPM'   .format( ppm_sw                 )   )
        self.txt['GAS HW'   ].set_text( '{:#.2f} PPM'   .format( self.sens.meas.ppm_hw  )   )
        self.txt['ADC'      ].set_text( '{:#4.2f} mV'   .format( adc_mV                 )   )
        self.txt['TEMP'     ].set_text( '{:#4.2f} °C'   .format( self.sens.meas.temp_digc ))
        #self.txt['PRES'     ].set_text( '{:#4.2f} hPa'  .format( self.sens.meas.pres_hpa ) )
        #self.txt['RMS'      ].set_text( '{:#4.2f}'      .format( rmse )                    )

        self.txt['PRES'     ].set_text( '%4.2f hPa' % self.sens.meas.pres_hpa   )
        self.txt['RMS'      ].set_text( '%4.2f'     % rmse                      )
        self.txt['MCU TEMP' ].set_text( '%d °C'     % self.sens.meas.mcu_digc   )
        self.txt['MCU VDDA' ].set_text( '%d mV'     % self.sens.meas.mcu_vdda   )


    def button( self, event, label ):
        if label == 'P SPAN':
            self.sens.trim_p1()
            self.conf[label]['ppm'] = str( self.sens.trim.ppm[ 1] )
            self.conf[label]['raw'] = str( self.sens.trim.raw[ 1] )
            self.conf_save()
            self.txt['P SPAN'].set_text( '%.2f / %.2f' % (sens.trim.ppm[ 1], sens.raw_to_mV( sens.trim.raw[ 1]) ) )

        elif label == 'P ZERO':
            self.sens.trim_p0()
            self.conf[label]['ppm'] = str( self.sens.trim.ppm[ 0] )
            self.conf[label]['raw'] = str( self.sens.trim.raw[ 0] )
            self.conf_save()
            self.txt['P ZERO'].set_text( '%.2f / %.2f' % (sens.trim.ppm[ 0], sens.raw_to_mV( sens.trim.raw[ 0]) ) )

        elif label == 'PS UPLOAD':
            resp    = self.sens.trim_write( 1 )
            self.txt['Ps UPLOAD'].set_text( resp )

        elif label == 'PZ UPLOAD':
            resp    = self.sens.trim_write( 0 )
            self.txt['Pz UPLOAD'].set_text( resp )

        elif label == 'K TEMP':
            self.sens.trim_drift_temp( self.sens.meas.temp_digc,  self.sens.y[-1] )
        elif label == 'K PRES':
            self.sens.trim_kpres( self.sens.meas.pres_hpa )

        elif label == 't0':
            self.sens.afe_drift_ktemp_set( 0, self.sens.meas.adc_raw, self.sens.meas.temp_digc )
            adc_mv  = self.sens.raw_to_mV( self.sens.meas.adc_raw )
            self.txt['t0'      ].set_text( '%4.2f mV @ %4.2f °C'   % (adc_mv, self.sens.ktemp.t0_digc) )

        elif label == 't1':
            self.sens.afe_drift_ktemp_set( 1, self.sens.meas.adc_raw, self.sens.meas.temp_digc )
            adc_mv  = self.sens.raw_to_mV( self.sens.ktemp.adc_1_raw )
            self.txt['t1'      ].set_text( '%4.2f mV @ %4.2f °C'   % (adc_mv, self.sens.ktemp.t1_digc) )

        elif label == 'Kt CALC':
            k = self.sens.afe_drift_ktemp_calc()
            self.txt['Kt CALC'].set_text( '%.8f' % k )

        elif label == 'Kt UPLOAD':
            resp    = self.sens.afe_drift_ktemp_upload( self.sens.afe_drift_ktemp_get() )
            self.txt['Kt UPLOAD'].set_text( resp )

        elif label == 'p0':
            self.sens.afe_drift_kpres_save( 0, self.sens.meas.adc_raw, self.sens.meas.pres_hpa )
            adc_mv  = self.sens.raw_to_mV( self.sens.meas.adc_raw )
            self.txt['p0'      ].set_text( '%4.2f mV @ %4.2f hPa'   % (adc_mv, self.sens.kpres.p0_hpa) )

        elif label == 'p1':
            self.sens.afe_drift_kpres_save( 1, self.sens.meas.adc_raw, self.sens.meas.pres_hpa )
            adc_mv  = self.sens.raw_to_mV( self.sens.meas.adc_raw )
            self.txt['p1'      ].set_text( '%4.2f mV @ %4.2f hPa'   % (adc_mv, self.sens.kpres.p1_hpa) )

        elif label == 'Kp CALC':
            k = self.sens.afe_drift_kpres_calc()
            self.txt['Kp CALC'].set_text( '%.8f' % k )

        elif label == 'Kp UPLOAD':
            resp    = self.sens.afe_drift_kpres_upload( self.sens.afe_drift_kpres_get() )
            self.txt['Kp UPLOAD'].set_text( resp )

        else:
            print( label, 'not implemented yet' )


    def conf_save( self ):
        confpath   = self.conf['DEFAULT']['filename']
        with open( confpath, "w" ) as configfile:
            self.conf.write( configfile )


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()
    conf['DEFAULT']['config_path'] = str('ganz.ini')
    conf.read( conf['DEFAULT']['config_path'] )

    ###########################################################################
    # SENSOR
    sens    = Sensor( conf )
    sens.read_config()

    ###########################################################################
    # FIGURE
    title   =   conf['MODBUS']['port'    ] + '@' + \
                conf['MODBUS']['baudrate'] + ' ADDR: ' +  \
                conf['MODBUS']['address' ] + ' S/N: ' + \
                sens.serial_number

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )

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
    (   'GAS PPM',  o2_min,         o2_max,         'blue',     'dashed',   1,      ),
    (   'ADC mV',   adc_mv_min,     adc_mv_max,     'orange',   'dotted',   1,      ),
    (   't DigC',   t_digc_min,     t_digc_max,     'red',      'dashed',   1,      ),
    (   'P hPa',    p_hpa_min,      p_hpa_max,      'green',    'dashdot',  1,      ), ]

    xlen    = conf.getint( 'GRAPH', 'axlen' )
    #ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.25], axes_class=HostAxes )
    ax      = fig.add_axes( [0.05, 0.05, 0.80, 0.25], axes_class=HostAxes )
    graph   = Graph( ax, xlen, param )
    graph.init_timestamp( graph.xdata )

    graph.ydata['GAS PPM'] = [0.0 for a in range( len(graph.ydata['GAS PPM']))]

    ###########################################################################
    # GUI
    #ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2     = fig.add_axes( [0.05, 0.35, 0.90, 0.60], axes_class=HostAxes, frame_on=False )

    #ax2.set_facecolor('xkcd:dark gray')
    #ax2.set_facecolor('gray')
    #axisbg='red'

    ax2.set(xticks=[], yticks=[])
    #ax2.axis["left","right","top","bottom"].set_visible(False)

    htxt        = {}

    ###########################################################################
    # SENS CONFIG
    txt_conf    = [ ('DEVICE ID',       'darkgray',     ),
                    ('HARDWARE ID',     'darkgray',     ),
                    ('FIRMWARE ID',     'darkgray',     ),
                    ('LAST ERROR',      'darkgray',     ),
                    ('STARTS COUNTER',  'darkgray',     ),
                    ('ADC SPAN',        'darkgray',     ),
                    ('ADC RESOLUTION',  'darkgray',     ),  ]

    xpos, ypos = 0.15, 0.90
    for key in txt_conf:
        ax2.text( xpos, ypos, key[ 0]+':  ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.05

    htxt['DEVICE ID'        ].set_text( sens.device_id                      )
    htxt['HARDWARE ID'      ].set_text( sens.hardware_id                    )
    htxt['FIRMWARE ID'      ].set_text( sens.firmware_id                    )
    htxt['LAST ERROR'       ].set_text( '%04Xh'     % sens.last_error       )
    htxt['STARTS COUNTER'   ].set_text( sens.starts_counter                 )
    htxt['ADC SPAN'         ].set_text( '%d mV'     % sens.adc_span         )
    htxt['ADC RESOLUTION'   ].set_text( '%d bit'    % sens.adc_resolution   )

    ###########################################################################
    # SENS MEASURE
    txt_meas    = [ ('GAS SW',          'blue',     ),
                    ('GAS HW',          'magenta',  ),
                    ('ADC',             'orange',   ),
                    ('TEMP',            'red',      ),
                    ('PRES',            'green',    ),
                    ('RMS',             'yellow',   ),
                    ('SLOPE',           'gray',     ),
                    ('MCU TEMP',        'darkgray',     ),
                    ('MCU VDDA',        'darkgray',     ),  ]

    xpos, ypos = 0.30, 0.90
    for key in txt_meas:
        ax2.text( xpos, ypos, key[ 0]+':  ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.05

    htxt['RMS'].set_backgroundcolor('gray')


    ###########################################################################
    # TRIM
    b_pspan_save    = Button( plt.axes([0.075, 0.55, 0.125, 0.05]), 'P SPAN'        )
    b_pspan_upld    = Button( plt.axes([0.075, 0.50, 0.125, 0.05]), 'PS UPLOAD'     )
    b_pzero_save    = Button( plt.axes([0.075, 0.45, 0.125, 0.05]), 'P ZERO'        )
    b_pzero_upld    = Button( plt.axes([0.075, 0.40, 0.125, 0.05]), 'PZ UPLOAD'     )
    b_pspan_save.on_clicked( lambda x: cbk.button(x, b_pspan_save.label.get_text()) )
    b_pspan_upld.on_clicked( lambda x: cbk.button(x, b_pspan_upld.label.get_text()) )
    b_pzero_save.on_clicked( lambda x: cbk.button(x, b_pzero_save.label.get_text()) )
    b_pzero_upld.on_clicked( lambda x: cbk.button(x, b_pzero_upld.label.get_text()) )

    txt_trim    = [ ('P SPAN',      'gray',     ),
                    ('Ps UPLOAD',   'gray',     ),
                    ('P ZERO',      'gray',     ),
                    ('Pz UPLOAD',   'gray',     ),  ]

    xpos, ypos = 0.175, 0.3625
    for key in txt_trim:
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color( key[ 1] )
        #htxt[ key[ 0] ].set_text( '{:#.2f} PPM'   .format( conf.getfloat(key[0], 'ppm') ) )
        ypos    -= 0.0825

    #htxt['P SPAN'       ].set_text( '%.2f / %.2f' % (sens.trim.ppm[ 1], sens.trim.raw[ 1])     )
    htxt['P SPAN'       ].set_text( '%.2f / %.2f' % (sens.trim.ppm[ 1], sens.raw_to_mV( sens.trim.raw[ 1]) ) )
    htxt['Ps UPLOAD'    ].set_text( '%.8f'      % sens.trim_span_read() )
    #htxt['P ZERO'       ].set_text( '%.2f / %.2f' % (sens.trim.ppm[ 0], sens.trim.raw[ 0])     )
    htxt['P ZERO'       ].set_text( '%.2f / %.2f' % (sens.trim.ppm[ 0], sens.raw_to_mV( sens.trim.raw[ 0]) ) )
    htxt['Pz UPLOAD'    ].set_text( '%.8f'      % sens.trim_zero_read() )

    ###########################################################################
    # AFE DRIFT Ktemp
    btn_t0  = Button( plt.axes([0.50, 0.85, 0.125, 0.05]), 't0'         )
    btn_t1  = Button( plt.axes([0.50, 0.80, 0.125, 0.05]), 't1'         )
    btn_tu  = Button( plt.axes([0.50, 0.75, 0.125, 0.05]), 'Kt CALC'    )
    btn_tl  = Button( plt.axes([0.50, 0.70, 0.125, 0.05]), 'Kt UPLOAD'  )

    btn_t0.on_clicked( lambda x: cbk.button(x, btn_t0.label.get_text()) )
    btn_t1.on_clicked( lambda x: cbk.button(x, btn_t1.label.get_text()) )
    btn_tu.on_clicked( lambda x: cbk.button(x, btn_tu.label.get_text()) )
    btn_tl.on_clicked( lambda x: cbk.button(x, btn_tl.label.get_text()) )

    txt_ktemp   = [ ('t0',          'gray',     ),
                    ('t1',          'gray',     ),
                    ('Kt CALC',     'gray',     ),
                    ('Kt UPLOAD',   'gray',     ),  ]

    xpos, ypos = 0.65, 0.8625
    for key in txt_ktemp:
        #ax2.text( xpos, ypos, key[0]+': ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color( key[ 1] )
        #htxt[ key[ 0] ].set_text( '{:#.2f} kt'   .format( conf.getfloat(key[0], 'ppm') ) )
        ypos    -= 0.0825

    htxt['t0'       ].set_text( '%4.2f mV @ %4.2f °C'   % (sens.raw_to_mV( sens.ktemp.adc_0_raw ), sens.ktemp.t0_digc) )
    htxt['t1'       ].set_text( '%4.2f mV @ %4.2f °C'   % (sens.raw_to_mV( sens.ktemp.adc_1_raw ), sens.ktemp.t1_digc) )
    htxt['Kt CALC'  ].set_text( '%.8f'                  % (sens.ktemp.ktemp)                                )
    htxt['Kt UPLOAD'].set_text( '%.8f'                  % sens.afe_drift_ktemp_read() )

    ###########################################################################
    # AFE DRIFT Kpres
    btn_p0  = Button( plt.axes([0.50, 0.55, 0.125, 0.05]), 'p0'         )
    btn_p1  = Button( plt.axes([0.50, 0.50, 0.125, 0.05]), 'p1'         )
    btn_pc  = Button( plt.axes([0.50, 0.45, 0.125, 0.05]), 'Kp CALC'    )
    btn_pu  = Button( plt.axes([0.50, 0.40, 0.125, 0.05]), 'Kp UPLOAD'  )

    btn_p0.on_clicked( lambda x: cbk.button(x, btn_p0.label.get_text()) )
    btn_p1.on_clicked( lambda x: cbk.button(x, btn_p1.label.get_text()) )
    btn_pc.on_clicked( lambda x: cbk.button(x, btn_pc.label.get_text()) )
    btn_pu.on_clicked( lambda x: cbk.button(x, btn_pu.label.get_text()) )

    txt_kpres   = [ ('p0',          'gray',     ),
                    ('p1',          'gray',     ),
                    ('Kp CALC',     'gray',     ),
                    ('Kp UPLOAD',   'gray',     ),  ]

    xpos, ypos = 0.65, 0.3625
    for key in txt_kpres:
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color( key[ 1] )
        ypos    -= 0.0825

    htxt['p0'       ].set_text( '%4.2f mV @ %4.2f hPa'  % (sens.raw_to_mV( sens.kpres.adc_0_raw ), sens.kpres.p0_hpa) )
    htxt['p1'       ].set_text( '%4.2f mV @ %4.2f hPa'  % (sens.raw_to_mV( sens.kpres.adc_1_raw ), sens.kpres.p1_hpa) )
    htxt['Kp CALC'  ].set_text( '%.8f'                  % (sens.kpres.kpres)            )
    htxt['Kp UPLOAD'].set_text( '%.8f'                  % sens.afe_drift_kpres_read()   )

    ###########################################################################
    # COLOR MAP
    #fig.patch.set_facecolor('#202020')
    #fig.patch.set_facecolor('black')
    fig.patch.set_facecolor('#F8F8F8')

    #ax.set_facecolor('xkcd:gray')
    #ax.set_facecolor('black')

    ###########################################################################
    # CALLBACK
    cbk     = Callback( conf, sens, graph, htxt )

    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer )
    timer.start()

    plt.show()
