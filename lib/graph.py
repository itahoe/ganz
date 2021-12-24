## -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import matplotlib.units as munits
from datetime import datetime, timedelta
from matplotlib.ticker import AutoMinorLocator, MultipleLocator
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from configparser import ConfigParser


###############################################################################
# GRAPH CFG
class   Graph_cfg:
    __slots__       = 'ymin', 'ymax', 'color', 'linestyle', 'linewidth'


###############################################################################
# GRAPH
class   Graph:

    def __init__( self, host, xlen, param, direction='forward' ):

        self.host       = host
        self.direction  = direction
        self.xdata      = [None] * xlen
        self.ydata      = {}

        self.cfg        = {}

        for line in param:
            self.first_ay               = line[ 0]
            break

        for key in param:
            self.ydata[key[ 0]]             = [None] * xlen
            self.cfg[key[ 0]]               = Graph_cfg()
            self.cfg[key[ 0]].ymin          = key[ 1]
            self.cfg[key[ 0]].ymax          = key[ 2]
            self.cfg[key[ 0]].color         = key[ 3]
            self.cfg[key[ 0]].linestyle     = key[ 4]
            self.cfg[key[ 0]].linewidth     = key[ 5]

        #######################################################################
        # X-axis
        formats = [ '%y',        # ticks are mostly years
                    '%b',        # ticks are mostly months
                    '%d',        # ticks are mostly days
                    '%H:%M',     # hrs
                    '%H:%M',     # min
                    '%S.%f', ]   # secs
        # these can be the same, except offset by one level....
        zero_formats = [''] + formats[:-1]
        # ...except for ticks that are mostly hours, then its nice to have month-day
        zero_formats[3] = '%d-%b'
        offset_formats = [  '',
                            '%Y',
                            '%b %Y',
                            '%d %b %Y',
                            '%d %b %Y',
                            '%d %b %Y %H:%M', ]

        converter = mdates.ConciseDateConverter( formats=formats, zero_formats=zero_formats, offset_formats=offset_formats )
        munits.registry[np.datetime64]  = converter
        munits.registry[datetime.date]  = converter
        munits.registry[datetime]       = converter

        #############################################################################
        # Parasite Axes
        self.host.axis['left','top','bottom'].set_visible(True)
        self.host.axis['right'].set_visible(False)

        self.par    = {}
        self.p      = {}
        xofst       = 0
        for k in self.cfg:
            self.par[k] = ParasiteAxes( self.host, sharex=self.host )
            self.host.parasites.append( self.par[k] )

            if k == self.first_ay:
                self.host.set_ylim( [ self.cfg[k].ymin, self.cfg[k].ymax ] )
                self.host.axis['left' ] = self.host.new_fixed_axis(loc='left')
            else:
                #print(k, self.cfg[k].color)
                self.par[k].set_ylim( [ self.cfg[k].ymin, self.cfg[k].ymax ] )
                self.par[k].axis["right"] = self.par[k].new_fixed_axis(loc='right',  offset=(xofst, 0)   )
                xofst   += 50

        self.host.xaxis.set_minor_locator(AutoMinorLocator( 4 ))
        self.host.yaxis.set_minor_locator(AutoMinorLocator( 5 ))
        self.host.grid( which='major', color='#CCCCCC', linestyle='--' )
        self.host.grid( which='minor', color='#CCCCCC', linestyle=':' )

        #############################################################################
        # graph init
        for k in self.cfg:
            if k is self.first_ay:
                self.host.set_ylabel( k)
                self.p[k],  = self.host.plot(   self.xdata,
                                                self.ydata[k],
                                                label   = k,
                                                c       = self.cfg[k].color,
                                                ls      = self.cfg[k].linestyle,
                                                lw      = self.cfg[k].linewidth, )
                self.host.axis["left" ].label.set_color( self.cfg[k].color )

            else:
                self.par[k].set_ylabel( k )
                self.p[k],  = self.par[k].plot( self.xdata,
                                                self.ydata[k],
                                                label   = k,
                                                c       = self.cfg[k].color,
                                                ls      = self.cfg[k].linestyle,
                                                lw      = self.cfg[k].linewidth, )
                self.par[k].axis["left" ].label.set_color( self.cfg[k].color )
                self.par[k].axis["right"].label.set_color( self.cfg[k].color )

        #############################################################################
        # graph legend
        if self.direction=='backward':
            plt.legend(loc='upper left')
        else:
            plt.legend(loc='upper right')


    def plot( self ):

        lims    = [ np.datetime64( self.xdata[0] ), np.datetime64( self.xdata[-1] ) ]
        self.host.set_xlim( lims )

        for k in self.cfg:
            self.p[k].set_data( self.xdata, self.ydata[k] )

        plt.draw()


    def init_timestamp( self, data ):

        if self.direction=='backward':
            t = datetime.now() - timedelta(seconds=len(data))
            for i in range(len(data)):
                data[i] = t
                t += timedelta(seconds=1)
        else:
            t = datetime.now()
            for i in range(len(data)):
                data[i] = t
                t -= timedelta(seconds=1)


    def push( self, data, val ):

        if self.direction=='backward':
            data[:-1]   = data[1:]
            data[-1]    = val 
        else:
            data[1:]    = data[:-1]
            data[0]     = val


###############################################################################
# GUI CALLBACK
class Callback:

    def __init__( self, sens, graph ):
        self.sens   = sens
        self.g      = graph


    def timer( self ):
        m       = self.sens.read()
        ppm     = self.sens.raw_to_ppm( m['adc_raw'], m['t_digc'], m['p_hpa'] )
        adc_mV  = self.sens.raw_to_mV( m['adc_raw'] )

        self.g.push( self.g.xdata,                  datetime.now()  )
        self.g.push( self.g.ydata['PPM'       ],    ppm             )
        self.g.push( self.g.ydata['ADC RAW'   ],    m['adc_raw' ]   )
        self.g.push( self.g.ydata['ADC mV'    ],    adc_mV          )
        self.g.push( self.g.ydata['t DIGC'    ],    m['t_digc'  ]   )
        self.g.push( self.g.ydata['P hPa'     ],    m['p_hpa'   ]   )

        self.g.plot()


###############################################################################
# MAIN
if __name__ == '__main__':

    from o2mb_sensor    import Sensor

    ###########################################################################
    # CONFIG
    cfg     = ConfigParser()
    cfg['DEFAULT']['config_path']  = "ganz.ini"
    cfg.read( cfg['DEFAULT']['config_path'] )

    cfg_grph = ConfigParser()
    cfg_grph.read( "o2mb_graph.ini")


    title   =   cfg['SENSOR']['modbus_port'] + '@' +            \
                cfg['SENSOR']['modbus_baudrate'] + ' ADDR: ' +  \
                cfg['SENSOR']['modbus_address']

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( title )
    sens    = Sensor( cfg['SENSOR'] )

    ###########################################################################
    # GRAPH INIT
    param   = [
    #   name        ymin    ymax        color       linestyle   linewidth
    (   'PPM',      -50,    150,        'blue',     'dashed',   1,      ),
    (   'ADC RAW',  0,      10000000,   'orange',   'solid',    1,      ),
    (   'ADC mV',   0,      2500,       'cyan',     'solid',    1,      ),
    (   't DIGC',   15,     45,         'red',      'dashdot',  1,      ),
    (   'P hPa',    950,    1050,       'green',    'dotted',   1,      ), ]

    xlen    = cfg.getint( 'GRAPH', 'axlen' )
    ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.90], axes_class=HostAxes )
    graph   = O2mb_graph( ax, xlen, param )
    graph.init_timestamp( graph.xdata )

    ###########################################################################
    # TIMER
    cbk     = Callback( sens, graph )
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer )
    timer.start()

    plt.show()
