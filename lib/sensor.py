import numpy as np
import math
import sys
from scipy.signal   import kaiserord, butter, lfilter, lfilter_zi, filtfilt, firwin, freqz
from graph          import Graph
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
class Sensor:

    def __init__(self, cfg):

        self.hreg   = { 'AFE_STATUS'                :1024,
                        'AFE_CONTROL'               :1025,
                        'AFE_OFFSET'                :1026,
                        'AFE_GAIN'                  :1027,
                        'AFE_CHANNEL'               :1028,  }

        self.coil   = { 'AFE_CTL_INPUT_SHORT'       :0,
                        'AFE_CTL_UNIPOLAR'          :1,
                        'AFE_CTL_BUFFER_ENABLE'     :2,     }

        self.config     = dict.fromkeys([   'device_id',
                                            'hardware_id',
                                            'firmware_id',
                                            'serial_number' ])

        __slots__           = 'y', 'p',

        self.sts            = SensorStatus()
        self.meas           = SensorMeasure()
        self.ktemp          = SensorKtemp()
        self.kpres          = SensorKpres()

        self.meas.adc_raw   = None
        self.meas.adc_mV    = None
        self.meas.temp_digc = None
        self.meas.pres_hpa  = None

        ########################################################################
        # TRIM RESTORE
        self.trim           = SensorTrim()
        self.trim.raw       = [None] * 2
        self.trim.ppm       = [None] * 2
        self.trim.raw[ 0]   = cfg.getfloat('P ZERO', 'raw')
        self.trim.ppm[ 0]   = cfg.getfloat('P ZERO', 'ppm')
        self.trim.raw[ 1]   = cfg.getfloat('P HIGH', 'raw')
        self.trim.ppm[ 1]   = cfg.getfloat('P HIGH', 'ppm')

        self.trim.offset, self.trim.slope     = self.trim_update( self.trim )

        ########################################################################
        # K TEMPERATURE RESTORE
        #self.ktemp.raw_0    = cfg.getfloat('ktemp_raw_0')
        #self.ktemp.raw_1    = cfg.getfloat('ktemp_raw_1')
        #self.ktemp.digc_0   = cfg.getfloat('ktemp_digc_0')
        #self.ktemp.digc_1   = cfg.getfloat('ktemp_digc_1')
        #self.trim_ktemp_update()

        #self.k_temp         = cfg.getfloat('afe_drift_t')
        #self.afe_drift_t    = cfg.getfloat('afe_drift_t')

        ########################################################################
        # K PRESSURE RESTORE
        #self.kpres.raw_0    = cfg.getfloat('kpres_raw_0')
        #self.kpres.raw_1    = cfg.getfloat('kpres_raw_1')
        #self.kpres.hpa_0    = cfg.getfloat('kpres_hpa_0')
        #self.kpres.hpa_1    = cfg.getfloat('kpres_hpa_1')
        #self.kpres.k        = self.kpres_calc()

        ########################################################################
        # LPF INIT
        lpf_sample_rate_hz  = cfg.getfloat('LPF', 'sample_rate_hz')
        lpf_width_hz        = cfg.getfloat('LPF', 'width_hz')
        lpf_ripple_db       = cfg.getfloat('LPF', 'ripple_db')
        lpf_cutoff_hz       = cfg.getfloat('LPF', 'cutoff_hz')

        nyq_rate            = lpf_sample_rate_hz / 2.0
        N, beta             = kaiserord(lpf_ripple_db, lpf_width_hz/ nyq_rate)
        self.taps           = firwin(N, lpf_cutoff_hz / nyq_rate, window=('kaiser', beta))

        #b, a                = butter(15, 0.05)
        #self.lpf_b          = b
        #self.lpf_a          = a
        #self.lpf_xn         = [0] * len(b) * 15

        #self.lpf_xn         = [0] * N
        self.xn             = [0.0] * len(self.taps)
        self.x              = [0.0] * len(self.taps)
        self.tn             = [0.0] * len(self.taps)
        self.pn             = [0.0] * len(self.taps)

        print( '    N:', N )
        print( ' taps:', len(self.taps) )


        #######################################################################
        # MODBUS INIT
        self.modbus_address = cfg.getint('MODBUS', 'address')
        self.master     = modbus_rtu.RtuMaster( serial.Serial(  port        = cfg.get('MODBUS', 'port'),
                                                                baudrate    = cfg.getint('MODBUS', 'baudrate'),
                                                                bytesize=8,
                                                                parity='N',
                                                                stopbits=1,
                                                                xonxoff=0 )     )
        self.master.set_timeout( 2.0 )
        self.master.set_verbose( False )


    def get_afe_control(self):
        return self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['AFE_CONTROL'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h'                      )


    def get_offset(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['AFE_OFFSET'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h'                      )
        return int( data[0] )

    def get_gain(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['AFE_GAIN'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h' )
        return int( data[0] )


    def get_afe_input(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['AFE_CHANNEL'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h' )
        return int( data[0] )


    def slope_get(self, data):
        slope   = 0
        return slope


    def read_config(self):
        d = self.master.execute(    slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = 0,
                                    quantity_of_x       = 16,
                                    data_format         = '>hhhhhhhhhhhhhhhh' )

        self.config['device_id'     ]   = str(      d[ 0] + d[ 1])
        self.config['hardware_id'   ]   = str(      d[ 2] + d[ 3])
        self.config['firmware_id'   ]   = int(      d[ 4] + d[ 5] + d[ 6])
        self.config['serial_number' ]   = int(      d[ 8] + d[ 9] + d[10] + d[11] + d[12] + d[13] + d[14] + d[15])

        self.measure['error_code'   ]   = int(      d[ 0])

        return self.measure


    def read(self):
        d = self.master.execute(    slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = 16,
                                    quantity_of_x       = 32,
                                    data_format         = '>hhhhhhhhhhffffffhhiiihh' )

        self.sts.error_code     = int(      d[ 0])
        self.sts.starts_cnt     = int(      d[ 1])
        self.sts.adc_vref       = int(      d[ 4])
        self.sts.adc_res_bits   = int(      d[ 5])
        self.meas.mcu_digc      = int(      d[ 8])
        self.meas.mcu_vdda      = int(      d[ 9])
        self.sts.raw2ppm_fv     = float(    d[10])
        self.sts.raw2ppm_ft     = float(    d[11])
        self.sts.raw2ppm_fp     = float(    d[12])
        self.meas.ppm_hw        = float(    d[13])
        self.meas.temp_digc     = float(    d[14])
        self.meas.pres_hpa      = float(    d[15])
        self.meas.adc_raw       = int(      d[18])
        self.meas.temp_raw      = int(      d[19])
        self.meas.pres_raw      = int(      d[20])
        self.meas.slope_raw     = int(      d[21])
        self.meas.offset_raw    = int(      d[22])

        return self.meas, self.sts


    def print(self, data):
        print(  "%06Xh\t\t%3.2f\t\t%3.2f\t\t%4.2f\t\t%f" %
                (   data['adc_raw'],
                    self.raw_to_mV( data['adc_raw'] ),
                    data['t_digc' ],
                    data['p_hpa'  ],
                    self.raw_to_ppm(data['adc_raw'], data['t_digc'], data['p_hpa']),    ),
                end='\r' )


    def raw_to_mV(self, raw):
        return float(raw) * (2500/(2**24))


    def raw_to_ppm(self, raw, t_digc, hpa ):
        self.xn[1:]     = self.xn[:-1]
        self.xn[0]      = raw

        self.tn[1:]     = self.tn[:-1]
        self.tn[0]      = t_digc

        #t               = lfilter(self.taps, 1.0, self.tn )
        #t_offset        = self.afe_drift_t * t_digc
        #self.t          = t[-1] + t_offset

        #p               = lfilter(self.taps, 1.0, self.pn )
        #p_offset        = self.kpres.k * hpa

        y               = lfilter(self.taps, 1.0, self.xn )
        #self.y          = y[-1] + self.t - p_offset
        self.y          = y[-1]

        #print( 'type(self.trim.offset): ', type(self.trim.offset) )
        ppm = self.trim.offset + (self.trim.slope * self.y)

        return( ppm )


    def trim_drift_temp( self, digc, raw ):
        self.ktemp.raw_1        = self.ktemp.raw_0
        self.ktemp.raw_0        = raw
        self.ktemp.digc_1       = self.ktemp.digc_0
        self.ktemp.digc_0       = digc
        self.trim_ktemp_update()


    def trim_ktemp_update(self):
        if (self.ktemp.raw_0 - self.ktemp.raw_1) != 0.0:
            self.ktemp.k            = (self.ktemp.digc_0 - self.ktemp.digc_1) / (self.ktemp.raw_0 - self.ktemp.raw_1)


    def kpres_update( self, raw, hpa ):
        #print('trim_kpres: ', hpa )
        self.kpres.raw_1        = self.kpres.raw_0
        self.kpres.raw_0        = raw
        self.kpres.hpa_1        = self.kpres.hpa_0
        self.kpres.hpa_0        = hpa
        self.kpres.k            = self.kpres_calc()


    def kpres_calc( self ):
        if (self.kpres.raw_0 - self.kpres.raw_1) != 0:
            return (self.kpres.hpa_0 - self.kpres.hpa_1) / (self.kpres.raw_0 - self.kpres.raw_1)
        return 0            


    def rmse( self, data ):
        average = 0
        xsum    = 0
        for x in data:   average += x
        average /= len( data )
        for x in data:   xsum += (x - average)**2
        xsum    /= ( len( data ) - 1)
        return math.sqrt( xsum )

    def trim_p0(self):
        self.trim.raw[ 0] = self.y
        self.trim.slope, self.trim.offset = self.trim_update( self.trim )


    def trim_p1(self):
        self.trim.raw[ 1] = self.y
        self.trim.slope, self.trim.offset = self.trim_update( self.trim )


    def trim_update( self, trim ):
        if (trim.raw[ 1] - trim.raw[ 0]) != 0.0:
            trim.slope  = (trim.ppm[ 1] - trim.ppm[ 0]) / (trim.raw[ 1] - trim.raw[ 0])
            trim.offset = trim.ppm[ 1] - (trim.raw[ 1] * trim.slope)
        else:
            trim.slope   = 1.0
            trim.offset  = 0.0

        print( trim.raw )
        print( trim.ppm )
        print( trim.slope, trim.offset )

        return trim.slope, trim.offset

    #def trim_save( self, confpath ):
    #    with open( confpath, "w" ) as configfile:
    #        self.cfg.write( configfile )




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

        average = 0
        xsum    = 0
        for x in self.graph.ydata['O2']:   average += x
        average /= len(self.graph.ydata['O2'])
        for x in self.graph.ydata['O2']:   xsum += (x - average)**2
        xsum    /= ( len(self.graph.ydata['O2']) - 1)
        rmse = math.sqrt( xsum )


        self.graph.push( self.graph.xdata,                  datetime.now()  )
        self.graph.push( self.graph.ydata['O2'        ],    ppm             )
        self.graph.push( self.graph.ydata['ADC RAW'   ],    meas.adc_raw    )
        self.graph.push( self.graph.ydata['ADC mV'    ],    adc_mV          )
        self.graph.push( self.graph.ydata['t DIGC'    ],    meas.temp_digc  )
        self.graph.push( self.graph.ydata['P hPa'     ],    meas.pres_hpa   )

        self.graph.plot()

        txt['O2'    ].set_text( '{:#.2f} PPM'   .format( ppm                            ) )
        txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( sens.raw_to_mV(meas.adc_raw)   ) )
        txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( meas.temp_digc                 ) )
        txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( meas.pres_hpa                  ) )
        txt['RMS'   ].set_text( '{:#4.2f}'      .format( rmse ) )
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

    ###########################################################################
    # CALLBACK
    cbk     = Callback( conf, sens, graph )

    ###########################################################################
    # GUI
    ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2.set(xticks=[], yticks=[])
    ax2.axis["left","right","top","bottom"].set_visible(False)

    hbtn_ph  = Button( plt.axes([0.3, 0.90, 0.20, 0.05]), 'P HIGH'    )
    hbtn_p3  = Button( plt.axes([0.3, 0.85, 0.20, 0.05]), 'P 3'       )
    hbtn_p2  = Button( plt.axes([0.3, 0.80, 0.20, 0.05]), 'P 2'       )
    hbtn_p1  = Button( plt.axes([0.3, 0.75, 0.20, 0.05]), 'P 1'       )
    hbtn_pz  = Button( plt.axes([0.3, 0.70, 0.20, 0.05]), 'P ZERO'    )

    hbtn_ph.on_clicked( lambda x: cbk.button(x, hbtn_ph.label.get_text()) )
    hbtn_p3.on_clicked( lambda x: cbk.button(x, hbtn_p3.label.get_text()) )
    hbtn_p2.on_clicked( lambda x: cbk.button(x, hbtn_p2.label.get_text()) )
    hbtn_p1.on_clicked( lambda x: cbk.button(x, hbtn_p1.label.get_text()) )
    hbtn_pz.on_clicked( lambda x: cbk.button(x, hbtn_pz.label.get_text()) )

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
