import numpy as np
import math
#import sys
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

class   SensorTitle:
    __slots__       =   'title',

class   SensorConfig:
    __slots__       =   'device_id', 'hardware_id', 'firmware_id', 'serial_number', \
                        'last_error', 'starts_counter', 'adc_span', 'adc_resolution' \
                        'adc_mv_per_bit'

class   SensorStatus:
    __slots__       =   'error_code', 'starts_cnt',                         \
                        'adc_vref', 'adc_res_bits',                         \
                        'raw2ppm_fv', 'raw2ppm_ft', 'raw2ppm_fp',

class   SensorMeasure:
    __slots__       =   'adc_raw', 'adc_mV', 'ppm_sw', 'ppm_hw',            \
                        'temp_digc', 'pres_hpa',                            \
                        'mcu_digc', 'mcu_vdda',                             \
                        'temp_raw', 'pres_raw', 'slope_raw', 'offset_raw',

class   SensorTrim:
    __slots__       =    'slope', 'offset', 'raw', 'ppm', 'timestamp',

class   SensorAfeDriftKtemp:
    __slots__       =   'ktemp', \
                        'adc_0_raw', 't0_digc', 't0_raw', \
                        'adc_1_raw', 't1_digc', 't1_raw',

class   SensorAfeDriftKpres:
    __slots__       =   'kpres', \
                        'adc_0_raw', 'p0_hpa', 'p0_raw', \
                        'adc_1_raw', 'p1_hpa', 'p1_raw',


###############################################################################
# SENSOR
class Sensor:

    def __init__(self, cfg):

        self.cfg        = cfg

        self.hreg   = { 
                        'HREG_CONF_BEGIN'               : 0x0000,
                        #'HREG_CONF_BEGIN'               : 0x0100,
                        'HREG_TRIM'                     : 0x0030,
                        'HREG_TRIM_ZERO'                : 0x0030,
                        'HREG_TRIM_SPAN'                : 0x0038,

                        #'HREG_CONF_AD7799_OFFSET'       : 0x0156,
                        'HREG_CONF_AFE_BIAS'            : 0x0144,

                        'HREG_CONF_AFE_DRIFT_K_TEMP'    : 0x0148,
                        'HREG_CONF_AFE_DRIFT_K_PRES'    : 0x014C,
                        'HREG_CONF_AD7799_MODE'         : 0x0150,
                        'HREG_CONF_AD7799_CONF'         : 0x0151,
                        'HREG_CONF_AD7799_CHANNEL'      : 0x0154,
                        'HREG_CONF_AD7799_GAIN'         : 0x0155,
                        'HREG_MEAS_BEGIN'               : 0x0200,   }

        self.coil   = { 'AFE_CTL_UNIPOLAR'          :0,
                        'AFE_CTL_BUFFER_ENABLE'     :1,
                        'AFE_CTL_PSWITCH'           :2,     }

        self.afe_chnl   = { 'AIN_1P'    :0,
                            'AIN_2P'    :1,
                            'AIN_3P'    :2,
                            'AIN_1N'    :3,
                            'RES_4'     :4,
                            'RES_5'     :5,
                            'RES_6'     :6,
                            'AVDD'      :7, }

        self.afe_gain_sel   = { 'GAIN x1'   :0,
                                'GAIN x2'   :1,
                                'GAIN x4'   :2,
                                'GAIN x8'   :3,
                                'GAIN x16'  :4,
                                'GAIN x32'  :5,
                                'GAIN x64'  :6,
                                'GAIN x128' :7, }

        self.config     = dict.fromkeys([   'device_id',
                                            'hardware_id',
                                            'firmware_id',
                                            'serial_number' ])

        self.device_id      = [None] * 4
        #self.hardware_id    = [None] * 4

        __slots__           = 'y', 'p',

        #self.config         = SensorConfig()
        self.sts            = SensorStatus()
        self.meas           = SensorMeasure()
        self.ktemp          = SensorAfeDriftKtemp()
        self.kpres          = SensorAfeDriftKpres()

        self.meas.adc_raw   = None
        self.meas.adc_mV    = None
        self.meas.temp_digc = None
        self.meas.pres_hpa  = None

        ########################################################################
        # TRIM RESTORE
        self.trim               = SensorTrim()
        self.trim.raw           = [None] * 2
        self.trim.ppm           = [None] * 2
        self.trim.timestamp     = [None] * 2
        self.trim.raw[       0] = cfg.getfloat( 'P ZERO', 'raw'         )
        self.trim.ppm[       0] = cfg.getfloat( 'P ZERO', 'ppm'         )
        self.trim.timestamp[ 0] = cfg.getint(   'P ZERO', 'timestamp'   )
        self.trim.raw[       1] = cfg.getfloat( 'P SPAN', 'raw'         )
        self.trim.ppm[       1] = cfg.getfloat( 'P SPAN', 'ppm'         )
        self.trim.timestamp[ 1] = cfg.getint(   'P ZERO', 'timestamp'   )

        self.trim.slope, self.trim.offset   = self.trim_update( self.trim )

        ########################################################################
        # AFE DRIFT BY TEMP RESTORE
        #self.afe_drift.adc_0_raw    = cfg.getint(   'DRIFT', 'adc_0_raw'    )
        self.ktemp.adc_0_raw    = int(cfg['KTEMP']['adc_0_raw'])
        self.ktemp.adc_1_raw    = cfg.getint(   'KTEMP', 'adc_1_raw'    )
        self.ktemp.t0_digc      = cfg.getfloat( 'KTEMP', 't0_digc'  )
        self.ktemp.t1_digc      = cfg.getfloat( 'KTEMP', 't1_digc'  )
        self.ktemp.ktemp        = self.afe_drift_ktemp_calc()

        ########################################################################
        # AFE DRIFT BY PRESSURE RESTORE
        self.kpres.adc_0_raw    = int(cfg['KPRES']['adc_0_raw'])
        self.kpres.adc_1_raw    = cfg.getint(   'KPRES', 'adc_1_raw'    )
        self.kpres.p0_hpa       = cfg.getfloat( 'KPRES', 'p0_hpa'  )
        self.kpres.p1_hpa       = cfg.getfloat( 'KPRES', 'p1_hpa'  )
        self.kpres.kpres        = self.afe_drift_kpres_calc()

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

        #print( '    N:', N )
        #print( ' taps:', len(self.taps) )


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

        self.read_config()

        self.title  = self.cfg.get('MODBUS', 'port') + '@'
        self.title  += self.cfg.get('MODBUS', 'baudrate') + ' ADDR: '
        self.title  += self.cfg.get('MODBUS', 'address')

    ###########################################################################
    # READ TRIM
    def read_trim( self ):
        fmt = '>I f f hh I f f hh'
        d = self.master.execute(    slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_TRIM'],
                                    quantity_of_x       = 16,
                                    data_format         = fmt )

        self.trim.timestamp[ 0] = int(      d[ 0])
        self.trim.ppm[       0] = float(    d[ 1])
        self.trim.raw[       0] = float(    d[ 2])

        self.trim.timestamp[ 1] = int(      d[ 5])
        self.trim.ppm[       1] = float(    d[ 6])
        self.trim.raw[       1] = float(    d[ 7])


    ###########################################################################
    # TRIM
    def trim_zero_read(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_TRIM_ZERO'],
                                    quantity_of_x       = 8,
                                    data_format         = '>I f f hh' )

        timestamp   = data[ 0]
        ppm         = data[ 1]
        raw         = data[ 2]

        #return ppm, raw, timestamp
        return ppm


    def trim_span_read(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_TRIM_SPAN'],
                                    quantity_of_x       = 8,
                                    data_format         = '>I f f hh' )

        timestamp   = data[ 0]
        ppm         = data[ 1]
        raw         = data[ 2]

        #return ppm, raw, timestamp
        return ppm


    ###########################################################################
    # AFE OFFSET
    def afe_offset_get(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AFE_BIAS'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h'                      )
        return int( data[0] )


    def afe_offset_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_REGISTER,
                                    starting_address    = self.hreg['HREG_CONF_AFE_BIAS'],
                                    output_value        = val )


    ###########################################################################
    # AFE ADC MODE
    def afe_adc_mode_get(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_MODE'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h'                      )
        return int( data[0] )


    def afe_adc_mode_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_REGISTER,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_MODE'],
                                    output_value        = val )

    ###########################################################################
    # AFE ADC CONF
    def afe_adc_conf_get(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_CONF'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h'                      )

        return int( data[0] )


    def afe_adc_conf_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_REGISTER,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_CONF'],
                                    output_value        = val )

    ###########################################################################
    # AFE ADC CHANNEL
    def afe_adc_channel_get(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_CHANNEL'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h' )

        return int(data[0])


    def afe_adc_channel_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_REGISTER,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_CHANNEL'],
                                    output_value        = val )


    ###########################################################################
    # AFE ADC GAIN
    def afe_adc_gain_get(self):
        data = self.master.execute( slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_GAIN'],
                                    quantity_of_x       = 1,
                                    data_format         = '>h' )
        return int( data[0] )


    def afe_adc_gain_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_REGISTER,
                                    starting_address    = self.hreg['HREG_CONF_AD7799_GAIN'],
                                    output_value        = val )


    ###########################################################################
    # AFE CONTROL
    def afe_ctl_unipolar_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_COIL,
                                    starting_address    = self.coil['AFE_CTL_UNIPOLAR'],
                                    output_value        = int(1) if val else int(0) )

    def afe_ctl_buffer_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_COIL,
                                    starting_address    = self.coil['AFE_CTL_BUFFER_ENABLE'],
                                    output_value        = int(1) if val else int(0) )

    def afe_ctl_pswitch_set(self, val):
        self.master.execute(        slave               = self.modbus_address,
                                    function_code       = cst.WRITE_SINGLE_COIL,
                                    starting_address    = self.coil['AFE_CTL_PSWITCH'],
                                    output_value        = int(1) if val else int(0) )

    ###########################################################################
    # SLOPE
    def slope_get(self, data):
        slope   = 0
        return slope

    ###########################################################################
    # READ CONFIG
    def read_config(self):
        fmt = '>BB BB hh BBBB hh HHHHHHHH H H hh H H hh H H hhhhhh'

        d   = self.master.execute(
                slave               = self.modbus_address,
                function_code       = cst.READ_HOLDING_REGISTERS,
                starting_address    = self.hreg['HREG_CONF_BEGIN'],
                quantity_of_x       = 32,
                data_format         = fmt )


        self.device_id      = str( '%02X%02X' % (d[ 0], d[ 1]) )
        self.hardware_id    = str( '%02X%02X' % (d[ 2], d[ 3]) )
        #self.firmware_id    = str( '%02X%02X%02X%02X%02X%02X' % (d[ 6], d[ 7], d[ 8], d[ 9], d[10], d[11]) )
        self.firmware_id    = str( '%02X%02X%02X%02X' % (d[ 6], d[ 7], d[ 8], d[ 9]) )

        self.serial_number  =   f'{d[12]:04X}{d[13]:04X}{d[14]:04X}{d[15]:04X}' + \
                                f'{d[16]:04X}{d[17]:04X}{d[18]:04X}{d[19]:04X}'

        self.last_error     = d[20]
        self.starts_counter = d[21]
        self.adc_span       = d[24]
        self.adc_resolution = d[25]
        self.adc_mv_per_bit = float( self.adc_span / (2**self.adc_resolution) )


    ###########################################################################
    # READ MEASURE
    def read_measure(self):
        fmt = '>fff hh h h hh h h hh I I I hhhhhhhhhh'
        d = self.master.execute(    slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_MEAS_BEGIN'],
                                    quantity_of_x       = 32,
                                    data_format         = fmt )

        self.meas.ppm_hw        = float(    d[ 0])
        self.meas.temp_digc     = float(    d[ 1])
        self.meas.pres_hpa      = float(    d[ 2])
        self.meas.slope_raw     = int(      d[ 5])
        self.meas.offset_raw    = int(      d[ 6])
        self.meas.mcu_digc      = int(      d[ 9])
        self.meas.mcu_vdda      = int(      d[10])

        self.meas.adc_raw       = int(      d[13])
        self.meas.temp_raw      = int(      d[14])
        self.meas.pres_raw      = int(      d[15])


    ###########################################################################
    # READ
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


    '''
    def print(self, data):
        print(  "%06Xh\t\t%3.2f\t\t%3.2f\t\t%4.2f\t\t%f" %
                (   data['adc_raw'],
                    self.raw_to_mV( data['adc_raw'] ),
                    data['temp_digc' ],
                    data['pres_hpa'  ],
                    self.raw_to_ppm(data['adc_raw'], data['temp_digc'], data['pres_hpa']),    ),
                end='\r' )
    '''


    ###########################################################################
    # RAW 2 mV
    def raw_to_mV(self, raw):
        return float(raw * self.adc_mv_per_bit)


    ###########################################################################
    # RAW 2 PPM
    def raw_to_ppm(self, raw, t_digc, hpa ):


        self.xn[1:]     = self.xn[:-1]
        self.xn[0]      = raw
        #self.xn[0]      = raw + self.ktemp.ktemp + self.kpres.kpres

        #self.tn[1:]     = self.tn[:-1]
        #self.tn[0]      = t_digc

        y               = lfilter(self.taps, 1.0, self.xn )
        self.y          = y[-1]
        ppm = self.trim.offset + (self.trim.slope * self.y)
        #ppm = self.trim.slope * (self.y + self.trim.offset)

        return( ppm )

    ###########################################################################
    # TRIM
    def trim_p0(self):
        self.trim.raw[ 0] = self.y
        self.trim.slope, self.trim.offset = self.trim_update( self.trim )

    def trim_p1(self):
        self.trim.raw[ 1] = self.y
        self.trim.slope, self.trim.offset = self.trim_update( self.trim )

    def trim_update( self, trim ):

        #print( 'trim_update', self.meas.adc_raw, trim.raw[ 0], trim.raw[ 1] )

        if (trim.raw[ 1] - trim.raw[ 0]) != 0.0:
            trim.slope  = (trim.ppm[ 1] - trim.ppm[ 0]) / (trim.raw[ 1] - trim.raw[ 0])
            trim.offset = trim.ppm[ 1] - (trim.raw[ 1] * trim.slope)
        else:
            trim.slope   = 1.0
            trim.offset  = 0.0

        #print( 'slope\toffset\t\tppm[ 0]\t\tppm[ 1]\t\traw[ 0]\t\traw[ 1]')
        #print( '%.4f\t' % trim.slope, '%.4f\t' % trim.offset, '%.2f\t\t' % trim.ppm[ 0], '%.2f\t\t' % trim.ppm[ 1], '%.2f\t' % trim.raw[ 0], '%.2f\t' % trim.raw[ 1] )

        return trim.slope, trim.offset

    #def trim_save( self, confpath ):
    #    with open( confpath, "w" ) as configfile:
    #        self.cfg.write( configfile )


    def trim_write(self, idx):
        if   idx == 0:
            timestamp   = self.trim.timestamp[ 0]
            ppm         = int( self.trim.ppm[ 0] )
            raw         = int( self.trim.raw[ 0] )
            try:
                self.master.execute(    slave               = self.modbus_address,
                                        function_code       = cst.WRITE_MULTIPLE_REGISTERS,
                                        starting_address    = self.hreg['HREG_TRIM_ZERO'],
                                        quantity_of_x       = 6,
                                        output_value        = [ timestamp, ppm, raw ],
                                        data_format         = '>III' )
                return 'Success'

            except Exception as err:
                return err

        elif idx == 1:
            timestamp   = self.trim.timestamp[ 1]
            ppm         = int( self.trim.ppm[ 1] )
            raw         = int( self.trim.raw[ 1] )

            try:
                self.master.execute(    slave               = self.modbus_address,
                                        function_code       = cst.WRITE_MULTIPLE_REGISTERS,
                                        starting_address    = self.hreg['HREG_TRIM_SPAN'],
                                        quantity_of_x       = 6,
                                        output_value        = [ timestamp, ppm, raw ],
                                        data_format         = '>III' )
                return 'Success'

            except Exception as err:
                return err

        else:
            return 'IDX out of range'


    def trim_read_raw(self, idx):
        pass

    ###########################################################################
    # AFE DRIFT BY TEMPERATURE
    def afe_drift_ktemp_set( self, idx, adc_raw, temp_digc ):

        if idx == 0:
            self.ktemp.adc_0_raw            = adc_raw
            self.ktemp.t0_digc              = temp_digc

            self.cfg['KTEMP']['adc_0_raw']  = str( adc_raw )
            self.cfg['KTEMP']['t0_digc']    = str( temp_digc )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
                self.cfg.write( configfile )

        elif idx == 1:
            self.ktemp.adc_1_raw            = adc_raw
            self.ktemp.t1_digc              = temp_digc

            self.cfg['KTEMP']['adc_1_raw']  = str( adc_raw )
            self.cfg['KTEMP']['t1_digc']    = str( temp_digc )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
                self.cfg.write( configfile )


    def afe_drift_ktemp_calc(self):
        t1      = self.ktemp.t1_digc
        t0      = self.ktemp.t0_digc
        raw1    = self.ktemp.adc_1_raw
        raw0    = self.ktemp.adc_0_raw
        k       = float(0)

        if (raw1 - raw0) != 0.0:
            k       = (t1 - t0) / (raw1 - raw0)
            self.ktemp.ktemp    = k
            #print( 'calc', k )

        return k


    def afe_drift_ktemp_get( self ):
        print( self.ktemp.ktemp )
        return( self.ktemp.ktemp )


    def afe_drift_ktemp_read( self ):
        d = self.master.execute(    slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AFE_DRIFT_K_TEMP'],
                                    quantity_of_x       = 2,
                                    data_format         = '>f' )

        print( d[ 0] )
        return( d[ 0] )


    def afe_drift_ktemp_upload( self, ktemp ):
        #print( ktemp )
        try:
            self.master.execute(        slave               = self.modbus_address,
                                        function_code       = cst.WRITE_MULTIPLE_REGISTERS,
                                        starting_address    = self.hreg['HREG_CONF_AFE_DRIFT_K_TEMP'],
                                        #starting_address    = 0x0350,
                                        quantity_of_x       = 2,
                                        output_value        = [ktemp],
                                        data_format         = '>f' )
            return 'Success'

        except Exception as err:
            return err

    ###########################################################################
    # AFE DRIFT BY PRESSURE
    def afe_drift_kpres_save( self, idx, adc_raw, pres_hpa ):

        if idx == 0:
            self.kpres.adc_0_raw            = adc_raw
            self.kpres.p0_hpa               = pres_hpa

            self.cfg['KPRES']['adc_0_raw']  = str( adc_raw )
            self.cfg['KPRES']['p0_hpa']     = str( pres_hpa )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
                self.cfg.write( configfile )

        elif idx == 1:
            self.kpres.adc_1_raw            = adc_raw
            self.kpres.p1_hpa               = pres_hpa

            self.cfg['KPRES']['adc_1_raw']  = str( adc_raw )
            self.cfg['KPRES']['p1_hpa']     = str( pres_hpa )
            with open( self.cfg['DEFAULT']['filename'], "w" ) as configfile:
                self.cfg.write( configfile )


    def afe_drift_kpres_calc(self):
        p1      = self.kpres.p1_hpa
        p0      = self.kpres.p0_hpa
        raw1    = self.kpres.adc_1_raw
        raw0    = self.kpres.adc_0_raw
        k       = float(0)

        if (raw1 - raw0) != 0.0:
            k       = (p1 - p0) / (raw1 - raw0)
            self.kpres.kpres    = k

        return k


    def afe_drift_kpres_get( self ):
        return( self.kpres.kpres )


    def afe_drift_kpres_read( self ):
        d = self.master.execute(    slave               = self.modbus_address,
                                    function_code       = cst.READ_HOLDING_REGISTERS,
                                    starting_address    = self.hreg['HREG_CONF_AFE_DRIFT_K_PRES'],
                                    quantity_of_x       = 2,
                                    data_format         = '>f' )

        return( d[ 0] )


    def afe_drift_kpres_upload( self, kpres ):
        try:
            self.master.execute(        slave               = self.modbus_address,
                                        function_code       = cst.WRITE_MULTIPLE_REGISTERS,
                                        starting_address    = self.hreg['HREG_CONF_AFE_DRIFT_K_PRES'],
                                        #starting_address    = 0x0350,
                                        quantity_of_x       = 2,
                                        output_value        = [kpres],
                                        data_format         = '>f' )
            return 'Success'

        except Exception as err:
            return err

    ###########################################################################
    # ROOT MEAN SQUARE ERROR
    def rmse( self, data ):
        average = 0
        xsum    = 0
        for x in data:   average += x
        average /= len( data )
        for x in data:   xsum += (x - average)**2
        xsum    /= ( len( data ) - 1)
        return math.sqrt( xsum )





###############################################################################
# CALLBACK
#class GuiCallback:
class Callback:


    def __init__(self, cfg, sens):
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


    def timer( self, txt ):
        meas, sts = self.sens.read()
        ppm     = self.sens.raw_to_ppm( meas.adc_raw, meas.temp_digc, meas.pres_hpa )
        adc_mV  = self.sens.raw_to_mV( meas.adc_raw )


        txt['O2'    ].set_text( '{:#.2f} PPM'   .format( ppm                            ) )
        txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( sens.raw_to_mV(meas.adc_raw)   ) )
        txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( meas.temp_digc                 ) )
        txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( meas.pres_hpa                  ) )

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
    conf['DEFAULT']['filename']  = "ganz.ini"
    conf.read( conf['DEFAULT']['filename'] )

    modbus = {  'port':     conf['MODBUS']['port'],
                'baud':     int(conf['MODBUS']['baudrate']),
                'addr':     int(conf['MODBUS']['address']),  }

    coef    = [float(s) for s in conf['SENSOR']['coefficients'].split(',')]
    print( 'coef: ', coef, '\tlen: ', len(coef), type(coef) )

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( modbus['port'] + '@' + str(modbus['baud']) + ' ' + str(modbus['addr']) )

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

    hbtn    = []

    for i in range(len(coef)):
        axes = plt.axes([0.3, 0.90 - (0.05 * i), 0.20, 0.05])
        hbtn.append( Button( axes, 'point '+str(i)    ) )
        hbtn[i].on_clicked( lambda x: cbk.button(x, hbtn[i].label.get_text()) )

    #hbtn0   = Button( plt.axes([0.3, 0.70, 0.20, 0.05]), 'P ZERO'    )
    #hbtn1   = Button( plt.axes([0.3, 0.75, 0.20, 0.05]), 'P 1'       )
    #hbtn2   = Button( plt.axes([0.3, 0.80, 0.20, 0.05]), 'P 2'       )
    #hbtn3   = Button( plt.axes([0.3, 0.85, 0.20, 0.05]), 'P 3'       )
    #hbtn4   = Button( plt.axes([0.3, 0.90, 0.20, 0.05]), 'P HIGH'    )

    #hbtn0.on_clicked( lambda x: cbk.button(x, hbtn0.label.get_text()) )
    #hbtn1.on_clicked( lambda x: cbk.button(x, hbtn1.label.get_text()) )
    #hbtn2.on_clicked( lambda x: cbk.button(x, hbtn2.label.get_text()) )
    #hbtn3.on_clicked( lambda x: cbk.button(x, hbtn3.label.get_text()) )
    #hbtn4.on_clicked( lambda x: cbk.button(x, hbtn4.label.get_text()) )

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


    print( "\r" )
    print( "|       ppm |   t\xB0C |     hPa |    slope |" )
    print( "-------------------------------------------------------------------------" )

    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer, htxt )
    timer.start()

    plt.show()
