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


class   DeviceModbus:
    __slots__       =   'master', 'port', 'baud', 'addr', 


###############################################################################
# CALLBACK
class Device:

    def __init__(self, cfg):
        self.device_id      = str('')
        self.hardware_id    = str('')
        self.firmware_id    = str('')
        self.serial_num     = str('')

        #######################################################################
        # MODBUS INIT
        self.modbus         = DeviceModbus()
        self.modbus.port    = cfg.get(      'MODBUS',   'port'      )
        self.modbus.baud    = cfg.getint(   'MODBUS',   'baudrate'  )
        self.modbus.addr    = cfg.getint(   'MODBUS',   'address'   )
        self.modbus.master  = None
        try:
            self.modbus.master  = modbus_rtu.RtuMaster( serial.Serial(  port    = self.modbus.port,
                                                                        baudrate = self.modbus.baud,
                                                                        bytesize=8,
                                                                        parity='N',
                                                                        stopbits=1,
                                                                        xonxoff=0 )     )
            self.modbus.master.set_timeout( 2.0 )
            self.modbus.master.set_verbose( False )
        except:
            print( 'Can not open %s' % self.modbus.port )

        #self.title  = 'Source: '
        #self.title  += cfg.get('MODBUS','port'      ) + '@'
        #self.title  += cfg.get('MODBUS','baudrate'  ) + ' ADDR: '
        #self.title  += cfg.get('MODBUS','address'   )


    def timer( self ):
        self.sens.read_config()
        self.txt['O2'    ].set_text( '{:#.2f} PPM'   .format( self.sens.meas.ppm_sw )   )
        self.txt['ADC'   ].set_text( '{:#4.2f} mV'   .format( self.sens.meas.adc_mV )   )
        self.txt['TEMP'  ].set_text( '{:#4.2f} °C'   .format( self.sens.meas.temp_digc ))
        self.txt['PRES'  ].set_text( '{:#4.2f} hPa'  .format( self.sens.meas.pres_hpa ) )
        self.txt['RMS'   ].set_text( '{:#4.2f}'      .format( rmse )                    )


    def modbus_read_hreg( self, start_addr, len, fmt ):
        if self.modbus.master != None:
            slave_addr  = self.modbus.addr
            fc          = cst.READ_HOLDING_REGISTERS
            try:
                d = self.modbus.master.execute( slave_addr, fc, start_addr, len, fmt )
                return d
            except modbus_tk.modbus.ModbusInvalidResponseError:
                print( 'Modbus Response Error' )


    def get_conf_head( self ):
        fmt = '>H H hh HH hh HHHHHHHH'
        d   = self.modbus_read_hreg( 0x0100, 16, fmt )
        if d != None:
            self.device_id      = str( '%04X' % d[ 0] )
            self.hardware_id    = str( '%04X' % d[ 1] )
            self.firmware_id    = str( '%04X%04X' % (d[ 4], d[ 5] ) )
            self.serial_num     = str( '%04X%04X%04X%04X%04X%04X%04X%04X' % (d[ 8],d[ 9],d[ 10],d[11],d[12],d[13],d[14],d[15]) )



###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()

    conf['DEFAULT']['config_path']  = str('ganz.ini')
    conf.read( conf['DEFAULT']['config_path'] )

    ###########################################################################
    # CHECK
    dev     = Device(conf)

    ###########################################################################
    # FIGURE
    fig     = plt.figure()

    ###########################################################################
    # GUI
    #ax2     = fig.add_axes( [0.05, 0.65, 0.90, 0.30], axes_class=HostAxes, frame_on=False )
    ax2     = fig.add_axes( [0.05, 0.05, 0.90, 0.90], axes_class=HostAxes, frame_on=False )
    ax2.set(xticks=[], yticks=[])

    htxt        = {}

    ###########################################################################
    # SENS CONFIG
    txt_conf    = [ ('DEVICE ID',       'darkgray',     ),
                    ('HARDWARE ID',     'darkgray',     ),
                    ('FIRMWARE ID',     'darkgray',     ),
                    ('LAST ERROR',      'darkgray',     ),
                    ('STARTS COUNTER',  'darkgray',     ),
                    ('ADC SPAN',        'darkgray',     ),
                    ('ADC RESOLUTION',  'darkgray',     ),
                    ('MCU TEMP',        'darkgray',     ),
                    ('MCU VDDA',        'darkgray',     ),    ]

    xpos, ypos = 0.15, 0.90
    for key in txt_conf:
        ax2.text( xpos, ypos, key[ 0]+':  ',                fontsize=10, horizontalalignment='right' )
        htxt[ key[ 0] ] = ax2.text( xpos, ypos, '', color=key[1],   fontsize=10, horizontalalignment='left' )
        htxt[ key[ 0] ].set_color(key[ 1])
        ypos    -= 0.05

    dev.get_conf_head()

    fig.canvas.manager.set_window_title( 'Serial Number: ' + dev.serial_num )

    htxt['DEVICE ID'        ].set_text( dev.device_id   )
    htxt['HARDWARE ID'      ].set_text( dev.hardware_id )
    htxt['FIRMWARE ID'      ].set_text( dev.firmware_id )

    #htxt['LAST ERROR'       ].set_text( '%04Xh'     % sens.sts.error_code   )
    #htxt['STARTS COUNTER'   ].set_text( sens.sts.starts_cnt                 )
    #htxt['ADC SPAN'         ].set_text( '%d mV'     % sens.conf.adc_vref    )
    #htxt['ADC RESOLUTION'   ].set_text( '%d bit'    % sens.conf.adc_bits    )




    ###########################################################################
    # TIMER
    #timer   = fig.canvas.new_timer(interval=1000)
    #timer.add_callback( cbk.timer )
    #timer.start()

    plt.show()
