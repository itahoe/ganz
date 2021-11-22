import sys
sys.path.append('../lib')

import time
import ctypes
import timeit
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons, CheckButtons, Slider
from matplotlib.ticker import AutoMinorLocator, MultipleLocator
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from datetime import datetime, timedelta
from sensor import Sensor
#from logger import Logger
from graph import Graph
#from o2mb_2127 import O2mb
from configparser import ConfigParser


def offset_increase(val):
    sA.set_val(sA.val+1)

def offset_decrease( val ):
    sA.set_val(sA.val-1)

def sens_p1_update(val):
    print( sens_p1_update )
    #sens.trim_p1( measure['lpf_raw'][0] )
    sens.trim_p1( sens.measure['adc_raw'] )
    cfg.update_sensor(sens.tg, sens.offset, sens.p0_raw, sens.p1_raw)

def sens_p0_update(val):
    print( sens_p0_update )
    sens.trim_p0( sens.measure['adc_raw'] )
    cfg.update_sensor(sens.tg, sens.offset, sens.p0_raw, sens.p1_raw)

def afe_control_cb(label):
    sts = afe_control.get_status()[ check_labels[label] ]

    if label=='SHORT INPUT':
        sens.set_afe_ctl_short_input(sts)

    if label=='UNIPOLAR':
        sens.set_afe_ctl_unipolar(sts)

    if label=='BUFFER ENABLE':
        sens.set_afe_ctl_buffer_enable(sts)

    plt.draw()

def offset_update(val):
    o2mb.set_offset( int(sA.val) )

def afe_gain_cb(label):
    o2mb.set_gain( afe_gain_dict[label] )

###############################################################################
# DISPLAY FORMAT
def callbackradio(label):
    o2mb.conc_fmt = label

###############################################################################
# GUI
class   Gui:

    def __init__(self, ax, cfg, sens):
        self.field  = { 'adc_raw':  '' }

        fs  = 10

        self.fld_adc_raw_label       = ax.text( 0.25, 0.95, 'ADC RAW:',             fontsize=fs, horizontalalignment='right' )
        self.fld_adc_raw             = ax.text( 0.30, 0.95, '-', color='green',     fontsize=fs, horizontalalignment='left' )
        self.fld_adc_mV_label        = ax.text( 0.25, 0.90, 'ADC mV:',              fontsize=fs, horizontalalignment='right' )
        self.fld_adc_mV              = ax.text( 0.30, 0.90, '-', color='green',     fontsize=fs, horizontalalignment='left' )

        self.fld_slope_label         = ax.text( 0.25, 0.80, 'SLOPE:',               fontsize=fs, horizontalalignment='right' )
        self.fld_slope               = ax.text( 0.30, 0.80, '-', color='green',     fontsize=fs, horizontalalignment='left' )
        self.fld_sens_temp_label     = ax.text( 0.25, 0.75, 'SENS TEMP:',           fontsize=fs, horizontalalignment='right' )
        self.fld_sens_temp           = ax.text( 0.30, 0.75, '-', color='red',       fontsize=fs, horizontalalignment='left' )
        self.fld_sens_pressure_label = ax.text( 0.25, 0.70, 'SENS PRES:',           fontsize=fs, horizontalalignment='right' )
        self.fld_sens_pressure       = ax.text( 0.30, 0.70, '-', color='green',     fontsize=fs, horizontalalignment='left' )
        self.fld_adc_vref_label      = ax.text( 0.25, 0.65, 'ADC VREF:',            fontsize=fs, horizontalalignment='right' )
        self.fld_adc_vref            = ax.text( 0.30, 0.65, '-', color='gray',      fontsize=fs, horizontalalignment='left' )
        self.fld_adc_res_label       = ax.text( 0.25, 0.60, 'ADC RES:',             fontsize=fs, horizontalalignment='right' )
        self.fld_adc_res             = ax.text( 0.30, 0.60, '-', color='gray',      fontsize=fs, horizontalalignment='left' )
        self.fld_afe_gain_label      = ax.text( 0.25, 0.55, 'AFE GAIN:',            fontsize=fs, horizontalalignment='right' )
        self.fld_afe_gain            = ax.text( 0.30, 0.55, '-', color='gray',      fontsize=fs, horizontalalignment='left' )
        self.fld_afe_chnl_label      = ax.text( 0.25, 0.50, 'AFE CHNL:',            fontsize=fs, horizontalalignment='right' )
        self.fld_afe_chnl            = ax.text( 0.30, 0.50, '-', color='gray',      fontsize=fs, horizontalalignment='left' )

        self.fld_sens_conc_label     = ax.text( 0.25, 0.40, 'SENS CONC:',           fontsize=fs, horizontalalignment='right' )
        self.fld_sens_conc           = ax.text( 0.30, 0.40, '-', color='blue',      fontsize=fs, horizontalalignment='left' )

        self.fld_mcu_temp_label      = ax.text( 0.25, 0.30,  'MCU TEMP:',           fontsize=fs, horizontalalignment='right' )
        self.fld_mcu_temp            = ax.text( 0.30, 0.30,  '-', color='green',    fontsize=fs, horizontalalignment='left' )
        self.fld_mcu_vdda_label      = ax.text( 0.25, 0.25,  'MCU VDDA:',           fontsize=fs, horizontalalignment='right' )
        self.fld_mcu_vdda            = ax.text( 0.30, 0.25,  '-', color='green',    fontsize=fs, horizontalalignment='left' )

        self.fld_modbus_port_label   = ax.text( 0.25, 0.15,  'MODBUS PORT:',        fontsize=fs, horizontalalignment='right' )
        self.fld_modbus_port         = ax.text( 0.30, 0.15,  '-', color='gray',     fontsize=fs, horizontalalignment='left' )
        self.fld_modbus_baud_label   = ax.text( 0.25, 0.10,  'MODBUS BAUD:',        fontsize=fs, horizontalalignment='right' )
        self.fld_modbus_baud         = ax.text( 0.30, 0.10,  '-', color='gray',     fontsize=fs, horizontalalignment='left' )
        self.fld_modbus_addr_label   = ax.text( 0.25, 0.05,  'MODBUS ADDR:',        fontsize=fs, horizontalalignment='right' )
        self.fld_modbus_addr         = ax.text( 0.30, 0.05,  '-', color='gray',     fontsize=fs, horizontalalignment='left' )

        ###############################################################################
        # BUTTONS
        axgainp = plt.axes([0.55, 0.425, 0.075, 0.05])
        axgainm = plt.axes([0.55, 0.375, 0.075, 0.05])
        ax_p1   = plt.axes([0.40, 0.425, 0.100, 0.05])
        ax_p0   = plt.axes([0.40, 0.375, 0.100, 0.05])

        self.btn_ofst_inc = Button(axgainp, 'OFST +')
        self.btn_ofst_inc.on_clicked( offset_increase )

        self.btn_ofst_dec = Button(axgainm, 'OFST -')
        self.btn_ofst_dec.on_clicked( offset_decrease )

        #btn_p1  = Button(ax_p1, f'P1: {sens.p1_ppm:.2f} PPM')
        #self.btn_p1  = Button(ax_p1, f'P1: {float(cfg["p1_ppm"]):.2f} PPM')
        self.btn_p1  = Button(ax_p1, f'P1: {float(cfg["P HIGH"]["ppm"]):.2f} PPM')
        self.btn_p1.on_clicked( sens_p1_update )

        #btn_p0  = Button(ax_p0, f'P0: {sens.p0_ppm:.2f} PPM')
        #self.btn_p0  = Button(ax_p0, f'P0: {float(cfg["p0_ppm"]):.2f} PPM')
        self.btn_p0  = Button(ax_p0, f'P0: {float(cfg["P ZERO"]["ppm"]):.2f} PPM')
        self.btn_p0.on_clicked( sens_p0_update )

        rax             = plt.axes([0.65, 0.35, 0.15, 0.30])
        afe_ctl         = sens.get_afe_control()
        check_labels    = {'SHORT INPUT':0,'UNIPOLAR':1,'BUFFER ENABLE':2,}
        check_actives   = ( bool(afe_ctl[0] & 1), bool(afe_ctl[0] & 2), bool(afe_ctl[0] & 4), )
        afe_control     = CheckButtons(rax, check_labels, actives=check_actives )
        afe_control.on_clicked(afe_control_cb)

        ###############################################################################
        # OFFSET
        offset_max = int(64)
        offset  = sens.get_offset()

        axcolor = 'lightgoldenrodyellow'
        axA = plt.axes([0.575, 0.55, 0.025, 0.35], facecolor=axcolor)

        sA = Slider(axA, 'OFFSET RAW', valmin=0, valmax=offset_max, orientation="vertical", valinit=offset, valstep=1)

        sA.on_changed(offset_update)

        ###############################################################################
        # DISPLAY FORMAT
        ax_dspl_fmt     = plt.axes([0.80, 0.35, 0.15, 0.30])
        self.dspl_fmt        = RadioButtons(ax_dspl_fmt, ('PPM', '%'))
        self.dspl_fmt.on_clicked(callbackradio)

        ###############################################################################
        # AFE GAIN
        ax_afe_gain     = plt.axes([0.80, 0.65, 0.15, 0.30])
        afe_gain_dict   = {'GAIN x1':0,'GAIN x2':1,'GAIN x4':2,'GAIN x8':3,'GAIN x16':4,'GAIN x32':5,'GAIN x64':6,'GAIN x128':7,}
        afe_gain        = RadioButtons(ax_afe_gain, afe_gain_dict.keys())
        afe_gain.set_active( sens.get_gain() )
        afe_gain.on_clicked(afe_gain_cb)

        ###############################################################################
        # AFE CHANNEL
        ax_afe_chnl         = plt.axes([0.65, 0.65, 0.15, 0.30])
        self.afe_chnl_dict  = {'AIN_1P':0, 'AIN_2P':1, 'AIN_3P':2, 'AIN_1N':3, 'AVDD':7,}
        afe_chnl            = RadioButtons( ax_afe_chnl, self.afe_chnl_dict.keys() )
        afe_chnl_idx        = sens.get_afe_input()
        #print( afe_chnl_idx )
        #afe_chnl.set_active( 1 )
        afe_chnl.set_active( afe_chnl_idx )
        afe_chnl.on_clicked( self.afe_channel_cb )

    def afe_channel_cb(self, idx):
        o2mb.set_afe_input( self.afe_chnl_dict[idx] )

    def update(self, data):
        #self.fld_adc_vref.set_text(      f'{a[ 4]} mV'                       )
        #self.fld_adc_res.set_text(       f'{a[ 5]} BITS'                     )
        #self.fld_afe_gain.set_text(      f'{1<<a[ 6]}'                       )
        #self.fld_afe_chnl.set_text(      f'{a[ 7]}'                          )
        #self.fld_mcu_temp.set_text(      f'{a[ 8]} °C'                       )
        #self.fld_mcu_vdda.set_text(      f'{a[ 9]} mV'                       )

        self.fld_adc_raw.set_text(       f'0x{data["adc_raw"]:06X}'       )
        self.fld_adc_mV.set_text(        f'{data["adc_mV"]:.6f} mV'       )
        #self.fld_slope.set_text(         f'{a[ 8]:d}'                        )
        self.fld_sens_temp.set_text(     f'{data["temp_digc"]:.2f} °C'    )
        self.fld_sens_pressure.set_text( f'{data["pres_hpa"]:4.2f} hPa'   )
        self.fld_sens_conc.set_text(     f'{data["ppm"]:.2f} PPM'         )


###############################################################################
# TIMER CALLBACK
def timer_cb( sens, gui, graph ):

    sens.read()
    sens.print( sens.measure )

    for k in graph.buf:
        graph.buf[k][1:]  = graph.buf[k][:-1]

    graph.buf['timestamp'][0]   = datetime.now()
    graph.buf['adc_raw'  ][0]   = sens.measure['adc_raw' ]
    graph.buf['t_digc'   ][0]   = sens.measure['t_digc'  ]
    graph.buf['p_hpa'    ][0]   = sens.measure['p_hpa'   ]
    graph.buf['ppm'      ][0]   = sens.raw_to_ppm( sens.measure['adc_raw'], sens.measure['t_digc'], sens.measure['p_hpa'] )
    graph.plot()

    '''
    self.measure['error_code'   ]   = int(      d[ 0])
    self.measure['starts_cnt'   ]   = int(      d[ 1])
    self.measure['adc_vref'     ]   = int(      d[ 4])
    self.measure['adc_res_bits' ]   = int(      d[ 5])
    self.measure['mcu_digc'     ]   = int(      d[ 8])
    self.measure['mcu_vdda'     ]   = int(      d[ 9])
    self.measure['raw2ppm_fv'   ]   = float(    d[10])
    self.measure['raw2ppm_ft'   ]   = float(    d[11])
    self.measure['raw2ppm_fp'   ]   = float(    d[12])
    self.measure['ppm'          ]   = float(    d[13])
    self.measure['t_digc'       ]   = float(    d[14])
    self.measure['p_hpa'        ]   = float(    d[15])
    self.measure['adc_raw'      ]   = int(      d[18])
    self.measure['t_raw'        ]   = int(      d[19])
    self.measure['p_raw'        ]   = int(      d[20])
    self.measure['slope_raw'    ]   = int(      d[21])
    self.measure['offset_raw'   ]   = int(      d[22])
    '''

    gui.fld_adc_raw.set_text(       '{:#06x}'.format( sens.measure['adc_raw'] ) )
    gui.fld_adc_mV.set_text(        '{:#4.2f} mV'.format( sens.raw_to_mV(sens.measure['adc_raw']) ) )
    #gui.fld_slope.set_text(         ''.format() )
    gui.fld_sens_temp.set_text(     '{:#4.2f}'.format( sens.measure['t_digc'        ] ) )
    gui.fld_sens_pressure.set_text( '{:#4.2f}'.format( sens.measure['p_hpa'         ] ) )
    gui.fld_adc_vref.set_text(      '{:#d} mV'.format( sens.measure['adc_vref'      ] ) )
    gui.fld_adc_res.set_text(       '{:#d}'   .format( sens.measure['adc_res_bits'  ] ) )
    #gui.fld_afe_gain.set_text(      ''.format() )
    #gui.fld_afe_chnl.set_text(      ''.format() )
    gui.fld_sens_conc.set_text(     '{:#4.2f}'.format( sens.raw_to_ppm( sens.measure['adc_raw'], sens.measure['t_digc'], sens.measure['p_hpa'] ) ) )
    gui.fld_mcu_temp.set_text(      '{:#4.2f}'.format( sens.measure['mcu_digc'   ] ) )
    gui.fld_mcu_vdda.set_text(      '{:#4.2f}'.format( sens.measure['mcu_vdda'   ] ) )


###############################################################################
# GRAPH BUFFER
def graph_buffer_create(axmax):
    buffer  = { 'timestamp':    [],
                'adc_raw':      [],
                't_digc':       [],
                'p_hpa':        [],
                'ppm':          []      }

    t = datetime.now()
    i = 0
    while i < axmax:
        buffer['timestamp'  ].append(t)
        buffer['adc_raw'    ].append(None)
        buffer['t_digc'     ].append(None)
        buffer['p_hpa'      ].append(None)
        buffer['ppm'        ].append(None)
        t = t - timedelta(seconds=1)
        i += 1

    return buffer


###############################################################################
# MAIN
if __name__ == '__main__':

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()

    conf['DEFAULT']['ini_path']     = str('../../ini/')
    conf['DEFAULT']['ini_name']     = str('ganz.ini')
    conf.read( conf['DEFAULT']['ini_path'] + conf['DEFAULT']['ini_name'] )

    title   = conf['MODBUS']['port'       ] + '@'         + \
              conf['MODBUS']['baudrate'   ] + ' ADDR: '   + \
              conf['MODBUS']['address'    ]

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )

    ###########################################################################
    # SENSOR
    sens    = Sensor( conf )

    ax1     = fig.add_axes( [ 0.05, 0.35,  0.60, 0.60 ] )
    ax1.set(xticks=[], yticks=[])
    ax1.set_title('SENSOR')

    gui     = Gui( ax1, conf, sens )

    print( 'adc_raw\t\tadc_mV\t\tt_digc\t\tp_hpa\t\tppm' )


    ###########################################################################
    # GRAPH INIT
    buffer  = graph_buffer_create( int( conf['GRAPH']['axlen'] ) )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.25], axes_class=HostAxes )
    #graph   = O2mb_graph( host, conf['GRAPH'], buffer )
    #graph.create()

    graph   = O2mb_graph( ax, conf['GRAPH'] )
    graph.create()
    graph.init_timestamp( graph.buf['timestamp'] )

    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( timer_cb, sens, gui, graph )
    timer.start()

    plt.show()
