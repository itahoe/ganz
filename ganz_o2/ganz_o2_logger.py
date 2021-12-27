import sys
sys.path.append('../lib')

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from datetime import datetime, timedelta
from configparser import ConfigParser
from sensor import Sensor
from graph import Graph


###############################################################################
# LOGGER 
class Logger:

    def create( self, filename, fields ):
        try:
            f           = open( filename, 'w', newline='', encoding='utf-8' )
            self.row    = dict.fromkeys( fields )
            self.writer = csv.writer( f )
            self.writer.writerow( dict.keys(self.row) )
            return self.row
        except IOError:
            return None

    def row_save( self ):
        self.writer.writerow( self.row.values() )

    def write( self, data ):
        for field in self.row:
            if data.get( field ):
                self.row[field] = data[field]
        self.writer.writerow( self.row.values() )

    def  get_fields( self, logpath ):
        with open(logpath, newline='') as csvfile:
            reader  = csv.DictReader(csvfile)
            header  = reader.fieldnames
        return header

    def read( self, filename, data ):
        with open(filename, newline='') as csvfile:
            reader  = csv.DictReader(csvfile)
            for row in reader:
                for idx in row:
                    try:
                        data[idx].append( row[idx] )
                    except:
                        pass


###############################################################################
# GUI CALLBACK
class Callback:

    def __init__( self, sens, graph, log, txt ):
        self.sens   = sens
        self.graph  = graph
        self.log    = log
        self.txt        = txt


    def timer( self ):
        self.sens.read_measure()
        self.sens.meas.adc_mV       = self.sens.raw_to_mV( self.sens.meas.adc_raw )
        self.sens.meas.ppm_sw       = self.sens.raw_to_ppm( self.sens.meas.adc_raw,
                                                            self.sens.meas.temp_digc,
                                                            self.sens.meas.pres_hpa )

        self.log.row['timestamp' ]   = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
        self.log.row['adc_raw'   ]   = self.sens.meas.adc_raw
        self.log.row['temp_digc' ]   = self.sens.meas.temp_digc
        self.log.row['pres_hpa'  ]   = self.sens.meas.pres_hpa
        self.log.row['ppm_sw'    ]   = self.sens.meas.ppm_sw
        self.log.row_save()

        #print( self.log.row['timestamp'], end='\t\r' )
        #self.sens.print( self.log.row )
        #print( self.log.row )

        self.graph.push( self.graph.xdata,             datetime.now()           )
        self.graph.push( self.graph.ydata['GAS PPM' ], self.sens.meas.ppm_sw    )
        #self.graph.push( self.graph.ydata['ADC RAW' ], self.sens.meas.adc_raw   )
        self.graph.push( self.graph.ydata['ADC mV'  ], self.sens.meas.adc_mV    )
        self.graph.push( self.graph.ydata['t digC'  ], self.sens.meas.temp_digc )
        self.graph.push( self.graph.ydata['P hPa'   ], self.sens.meas.pres_hpa  )

        self.graph.plot()

        rmse = self.sens.rmse( self.graph.ydata['GAS PPM'] )

        #self.txt['GAS SW'   ].set_text( '{:#.2f} PPM'   .format( self.sens.meas.ppm_sw  )   )
        self.txt['GAS SW'   ].set_text( '%.2f PPM'  % self.sens.meas.ppm_sw     )
        self.txt['GAS HW'   ].set_text( '%.2f PPM'  % self.sens.meas.ppm_hw     )
        self.txt['ADC'      ].set_text( '%4.2f mV'  % self.sens.meas.adc_mV     )
        self.txt['TEMP'     ].set_text( '%4.2f °C'  % self.sens.meas.temp_digc  )
        self.txt['PRES'     ].set_text( '%4.2f hPa' % self.sens.meas.pres_hpa   )
        self.txt['RMS'      ].set_text( '%4.2f'     % rmse                      )
        self.txt['MCU TEMP' ].set_text( '%d °C'     % self.sens.meas.mcu_digc   )
        self.txt['MCU VDDA' ].set_text( '%d mV'     % self.sens.meas.mcu_vdda   )


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()
    conf['DEFAULT']['config_path']  = str('ganz.ini')
    conf.read( conf['DEFAULT']['config_path'] )

    ###########################################################################
    # SENSOR
    sens    = Sensor( conf )

    log         = Logger()
    log_name    = '../../log/' + datetime.now().strftime('%Y%m%d.%H%M%S') + '.o2log'
    log.create( log_name, ['timestamp', 'adc_raw', 'temp_digc', 'pres_hpa', 'ppm_sw'] )

    ###########################################################################
    # FIGURE
    fig     = plt.figure()
    fig.canvas.manager.set_window_title( sens.title )

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
    (   'ADC mV',   adc_mv_min,     adc_mv_max,     'orangered','dashed',   1,      ),
    (   't digC',   t_digc_min,     t_digc_max,     'red',      'dashdot',  1,      ),
    (   'P hPa',    p_hpa_min,      p_hpa_max,      'green',    'dashdot',  1,      ), ]

    xlen    = conf.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.30, 0.05, 0.50, 0.90], axes_class=HostAxes )
    graph   = Graph( ax, xlen, param, direction='backward' )
    graph.init_timestamp( graph.xdata )

    graph.ydata['GAS PPM'] = [0.0 for a in range( len(graph.ydata['GAS PPM']))]


    ###########################################################################
    # GUI
    #ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    #ax2     = fig.add_axes( [0.05, 0.35, 0.90, 0.60], axes_class=HostAxes, frame_on=False )
    #ax2     = fig.add_axes( [0.05, 0.05, 0.25, 0.90], axes_class=HostAxes, frame_on=False )
    ax2     = fig.add_axes( [0.00, 0.00, 0.25, 1.0], axes_class=HostAxes, frame_on=False )

    #ax2.set_facecolor('xkcd:dark gray')
    #ax2.set_facecolor('gray')
    #axisbg='red'

    ax2.set(xticks=[], yticks=[])
    ax2.axis["left","right","top","bottom"].set_visible(False)

    htxt        = {}

    ###########################################################################
    # GUI SENS MEASURE
    txt_meas    = [ ('GAS SW',          'blue',     ),
                    ('GAS HW',          'magenta',  ),
                    ('ADC',             'orange',   ),
                    ('TEMP',            'red',      ),
                    ('PRES',            'green',    ),
                    ('RMS',             'yellow',   ),
                    ('SLOPE',           'gray',     ),
                    ('MCU TEMP',        'darkgray',     ),
                    ('MCU VDDA',        'darkgray',     ),  ]

    xpos, ypos = 0.50, 0.90
    for key in txt_meas:
        ax2.text( xpos, ypos, key[ 0]+':  ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '-', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.025

    htxt['RMS'].set_backgroundcolor('gray')

    ###########################################################################
    # GUI SENS CONFIG
    txt_conf    = [ ('DEVICE ID',       'darkgray',     ),
                    ('HARDWARE ID',     'darkgray',     ),
                    ('FIRMWARE ID',     'darkgray',     ),
                    ('LAST ERROR',      'darkgray',     ),
                    ('STARTS COUNTER',  'darkgray',     ),
                    ('ADC SPAN',        'darkgray',     ),
                    ('ADC RESOLUTION',  'darkgray',     ),  ]

    xpos, ypos = 0.50, 0.50
    for key in txt_conf:
        ax2.text( xpos, ypos, key[ 0]+':  ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.025

    htxt['DEVICE ID'        ].set_text( sens.device_id                      )
    htxt['HARDWARE ID'      ].set_text( sens.hardware_id                    )
    htxt['FIRMWARE ID'      ].set_text( sens.firmware_id                    )
    htxt['LAST ERROR'       ].set_text( '%04Xh'     % sens.sts.error_code   )
    htxt['STARTS COUNTER'   ].set_text( sens.sts.starts_cnt                 )
    htxt['ADC SPAN'         ].set_text( '%d mV'     % sens.conf.adc_vref    )
    htxt['ADC RESOLUTION'   ].set_text( '%d bit'    % sens.conf.adc_bits    )

    ###########################################################################
    # TIMER
    cbk     = Callback( sens, graph, log, htxt )
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer )
    timer.start()

    plt.show()
