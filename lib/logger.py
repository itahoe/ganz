import csv
import matplotlib.pyplot as plt
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from datetime import datetime, timedelta
from configparser import ConfigParser
from o2mb_sensor import Sensor
from o2mb_graph import O2mb_graph


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
# TIMER CALLBACK
def timer_cb( sens, graph, log ):

    measure = sens.read()
    sens.print( measure )


    log.row['timestamp' ]   = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
    log.row['adc_raw'   ]   = measure['adc_raw']
    log.row['t_digc'    ]   = measure['t_digc' ]
    log.row['p_hpa'     ]   = measure['p_hpa'  ]
    log.row['ppm'       ]   = measure['ppm'    ]
    log.row_save()

    print( log.row['timestamp'], end='\t' )
    sens.print( measure )

    #for k in graph.buf:
    #    graph.buf[k][1:]  = graph.buf[k][:-1]
    #graph.buf['timestamp'][0]   = datetime.now()
    #graph.buf['adc_raw'  ][0]   = measure['adc_raw']
    #graph.buf['t_digc'   ][0]   = measure['t_digc']
    #graph.buf['p_hpa'    ][0]   = measure['p_hpa']
    #graph.buf['ppm'      ][0]   = sens.raw_to_ppm( measure['adc_raw'], measure['t_digc'], measure['p_hpa'] )
    #graph.plot()

    self.g.push( self.g.xdata,                  datetime.now()  )
    self.g.push( self.g.ydata['PPM'       ],    ppm             )
    self.g.push( self.g.ydata['ADC RAW'   ],    m['adc_raw' ]   )
    self.g.push( self.g.ydata['ADC mV'    ],    adc_mV          )
    self.g.push( self.g.ydata['t DIGC'    ],    m['t_digc'  ]   )
    self.g.push( self.g.ydata['P hPa'     ],    m['p_hpa'   ]   )
    self.g.plot()


###############################################################################
# GUI CALLBACK
class Callback:

    def __init__( self, sens, graph, log ):
        self.sens   = sens
        self.graph  = graph
        self.log    = log

    def timer( self ):
        measure = self.sens.read()
        ppm     = self.sens.raw_to_ppm( measure['adc_raw'], measure['t_digc'], measure['p_hpa'] )
        adc_mV  = self.sens.raw_to_mV( measure['adc_raw'] )

        self.log.row['timestamp' ]   = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
        self.log.row['adc_raw'   ]   = measure['adc_raw']
        self.log.row['t_digc'    ]   = measure['t_digc' ]
        self.log.row['p_hpa'     ]   = measure['p_hpa'  ]
        self.log.row['ppm'       ]   = measure['ppm'    ]
        self.log.row_save()

        print( self.log.row['timestamp'], end='\t' )
        self.sens.print( measure )

        self.graph.push( self.graph.xdata,                  datetime.now()  )
        self.graph.push( self.graph.ydata['PPM'       ],    ppm             )
        self.graph.push( self.graph.ydata['ADC RAW'   ],    measure['adc_raw' ]   )
        self.graph.push( self.graph.ydata['ADC mV'    ],    adc_mV          )
        self.graph.push( self.graph.ydata['t DIGC'    ],    measure['t_digc'  ]   )
        self.graph.push( self.graph.ydata['P hPa'     ],    measure['p_hpa'   ]   )

        self.graph.plot()


###############################################################################
# MAIN
if __name__ == '__main__':

    cfg     = ConfigParser()
    cfgfile = "o2mb.ini"
    cfg.read( cfgfile )

    sens    = Sensor( cfg['SENSOR'] )

    print( 'timestamp\t\tadc_raw\t\tadc_mV\t\tt_digc\t\tp_hpa\t\tppm' )

    dt          = datetime.now()
    log_name    = 'log/' + dt.strftime('%Y%m%d.%H%M%S') + '.o2log'
    log         = Logger()
    log.create( log_name, ['timestamp', 'adc_raw', 't_digc', 'p_hpa', 'ppm'] )

    sens_name   =   cfg['SENSOR']['modbus_port'] + '@' + \
                    cfg['SENSOR']['modbus_baudrate']   + \
                ' ADDR: ' + cfg['SENSOR']['modbus_address']


    fig     = plt.figure()
    fig.canvas.manager.set_window_title( sens_name + ' - ' + log_name )


    ###########################################################################
    # GRAPH
    #host    = fig.add_axes( [0.05, 0.05, 0.70, 0.90], axes_class=HostAxes )
    #graph   = O2mb_graph( host, cfg['GRAPH'] )
    #graph.create()

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
    cbk     = Callback( sens, graph, log )
    timer   = fig.canvas.new_timer(interval=1000)
    timer.add_callback( cbk.timer )
    timer.start()

    plt.show()
