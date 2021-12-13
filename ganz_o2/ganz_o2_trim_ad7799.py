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



def sens_p1_update(val):
    print( sens_p1_update )
    #sens.trim_p1( measure['lpf_raw'][0] )
    sens.trim_p1( sens.measure['adc_raw'] )
    cfg.update_sensor(sens.tg, sens.offset, sens.p0_raw, sens.p1_raw)

def sens_p0_update(val):
    print( sens_p0_update )
    sens.trim_p0( sens.measure['adc_raw'] )
    cfg.update_sensor(sens.tg, sens.offset, sens.p0_raw, sens.p1_raw)


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

        #btn_p1  = Button(ax_p1, f'P1: {sens.p1_ppm:.2f} PPM')
        #self.btn_p1  = Button(ax_p1, f'P1: {float(cfg["p1_ppm"]):.2f} PPM')
        self.btn_p1  = Button(ax_p1, f'P1: {float(cfg["P HIGH"]["ppm"]):.2f} PPM')
        self.btn_p1.on_clicked( sens_p1_update )

        #btn_p0  = Button(ax_p0, f'P0: {sens.p0_ppm:.2f} PPM')
        #self.btn_p0  = Button(ax_p0, f'P0: {float(cfg["p0_ppm"]):.2f} PPM')
        self.btn_p0  = Button(ax_p0, f'P0: {float(cfg["P ZERO"]["ppm"]):.2f} PPM')
        self.btn_p0.on_clicked( sens_p0_update )


        #btn_ph  = Button( plt.axes([0.3, 0.90, 0.20, 0.05]), 'P HIGH'    )
        #btn_pz  = Button( plt.axes([0.3, 0.85, 0.20, 0.05]), 'P ZERO'    )
        #btn_ph.on_clicked( lambda x: cbk.button(x, btn_ph.label.get_text()) )
        #btn_pz.on_clicked( lambda x: cbk.button(x, btn_pz.label.get_text()) )



        ###############################################################################
        # OFFSET
        self.btn_ofst_inc = Button(axgainp, 'OFST +')
        self.btn_ofst_inc.on_clicked( self.offset_increase )

        self.btn_ofst_dec = Button(axgainm, 'OFST -')
        self.btn_ofst_dec.on_clicked( self.offset_decrease )

        offset_max = int(64)
        offset  = sens.afe_offset_get()

        axcolor = 'lightgoldenrodyellow'
        axA = plt.axes([0.575, 0.55, 0.025, 0.35], facecolor=axcolor)

        self.sA = Slider(axA, 'OFFSET RAW', valmin=0, valmax=offset_max, orientation="vertical", valinit=offset, valstep=1)

        self.sA.on_changed( self.offset_update )

        ###############################################################################
        # DISPLAY FORMAT
        ax_dspl_fmt         = plt.axes([0.80, 0.35, 0.15, 0.30])
        self.dspl_fmt       = RadioButtons(ax_dspl_fmt, ('PPM', '%'))
        self.dspl_fmt.on_clicked(callbackradio)


    def offset_increase( self, val ):
        self.sA.set_val( self.sA.val + 1 )


    def offset_decrease( self, val ):
        self.sA.set_val( self.sA.val - 1 )


    def offset_update( self, val ):
        sens.afe_offset_set( int( self.sA.val ) )


    def afe_gain_cbk(self, idx):
        sens.afe_adc_gain_set( idx )


    def afe_chnl_cbk( self, idx ):
        sens.afe_adc_channel_set( sens.afe_chnl[idx] )


    def afe_control_cbk(self, label):
        sts = afe_control.get_status()[ check_labels[label] ]

        if label=='UNIPOLAR':
            sens.afe_ctl_unipolar_set( sts )
        if label=='BUFFER':
            sens.afe_ctl_buffer_set( sts )
        if label=='P-SWITCH':
            sens.afe_ctl_pswitch_set( sts )



    def timer_cbk( self, sens, graph ):
        sens.read()
        sens.meas.adc_mV   = sens.raw_to_mV(    sens.meas.adc_raw )
        sens.meas.ppm_sw   = sens.raw_to_ppm(   sens.meas.adc_raw,
                                                sens.meas.temp_digc,
                                                sens.meas.pres_hpa )

        graph.push( graph.xdata,                  datetime.now()              )
        graph.push( graph.ydata['GAS'       ],    sens.meas.ppm_sw       )
        graph.push( graph.ydata['ADC mV'    ],    sens.meas.adc_mV       )
        graph.push( graph.ydata['t DigC'    ],    sens.meas.temp_digc    )
        graph.push( graph.ydata['P hPa'     ],    sens.meas.pres_hpa     )

        graph.plot()

        self.fld_adc_raw.set_text(      '%08X'      % sens.meas.adc_raw                 )
        self.fld_adc_mV.set_text(       '%4.2f mV'  % sens.raw_to_mV(sens.meas.adc_raw) )
        self.fld_sens_temp.set_text(    '%4.2f °C'  % sens.meas.temp_digc               )

        #gui.fld_sens_pressure.set_text( '{:#4.2f}'.format( sens.meas.pres_hpa       ) )
        self.fld_sens_pressure.set_text( '%4.2f hPa' % sens.meas.pres_hpa                )

        self.fld_adc_vref.set_text(      '{:#d} mV'.format( sens.sts.adc_vref        ) )
        self.fld_adc_res.set_text(       '{:#d}'   .format( sens.sts.adc_res_bits    ) )

        self.fld_sens_conc.set_text(     '{:#4.2f}'.format( sens.meas.ppm_sw         ) )
        self.fld_mcu_temp.set_text(      '{:#4.2f}'.format( sens.meas.mcu_digc       ) )
        self.fld_mcu_vdda.set_text(      '{:#4.2f}'.format( sens.meas.mcu_vdda       ) )


###############################################################################
# MAIN
if __name__ == '__main__':

    from configparser import ConfigParser

    ###########################################################################
    # CONFIG
    conf    = ConfigParser()

    conf['DEFAULT']['ini_name']     = str('ganz.ini')
    conf.read( conf['DEFAULT']['ini_name'] )

    title   = conf['MODBUS']['port'       ] + '@'         + \
              conf['MODBUS']['baudrate'   ] + ' ADDR: '   + \
              conf['MODBUS']['address'    ]

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )

    ###########################################################################
    # SENSOR
    sens    = Sensor( conf )
    sens.read_config()

    ###########################################################################
    # 
    ax1     = fig.add_axes( [ 0.05, 0.35,  0.60, 0.60 ] )
    ax1.set(xticks=[], yticks=[])
    ax1.set_title('SENSOR')

    gui     = Gui( ax1, conf, sens )

    #print( 'adc_raw\t\tadc_mV\t\tt_digc\t\tp_hpa\t\tppm' )


    ###########################################################################
    # GRAPH INIT
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
    (   'ADC mV',   adc_mv_min,     adc_mv_max,     'magenta',  'dotted',   1,      ),
    (   't DigC',   t_digc_min,     t_digc_max,     'red',      'dashed',   1,      ),
    (   'P hPa',    p_hpa_min,      p_hpa_max,      'green',    'dashdot',  1,      ), ]

    axlen   = conf.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.25], axes_class=HostAxes )
    graph   = Graph( ax, axlen, param )
    graph.init_timestamp( graph.xdata )

    ###############################################################################
    # AFE ADC CHANNEL
    ax_afe_chnl         = plt.axes([0.65, 0.65, 0.15, 0.30])
    afe_chnl            = RadioButtons( ax_afe_chnl, sens.afe_chnl.keys() )
    #afe_chnl_idx        = sens.afe_adc_channel_get()
    afe_chnl.set_active( sens.afe_adc_channel_get() )
    afe_chnl.on_clicked( gui.afe_chnl_cbk )

    ###############################################################################
    # AFE ADC GAIN
    afe_gain        = RadioButtons(plt.axes([0.80, 0.65, 0.15, 0.30]), sens.afe_gain_sel )
    afe_gain.set_active( sens.afe_adc_gain_get() )
    afe_gain.on_clicked( lambda idx: gui.afe_gain_cbk(sens.afe_gain_sel[idx]) )

    ###############################################################################
    # CONTROL
    rax             = plt.axes([0.65, 0.35, 0.15, 0.30])
    afe_adc_mode    = sens.afe_adc_mode_get()
    afe_adc_conf    = sens.afe_adc_conf_get()
    check_labels    = {'UNIPOLAR':0,'BUFFER':1,'P-SWITCH':2,}
    check_actives   = ( bool(afe_adc_conf & 0x1000), bool(afe_adc_conf & 0x0010), bool(afe_adc_mode & 0x1000), )
    afe_control     = CheckButtons(rax, check_labels, actives=check_actives )
    afe_control.on_clicked( gui.afe_control_cbk )
    #afe_control.on_clicked( lambda idx: gui.afe_control_cbk(sens.afe_gain_sel[idx]) )

    ###########################################################################
    # TIMER
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( gui.timer_cbk, sens, graph )

    timer.start()

    plt.show()
