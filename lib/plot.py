## -*- coding: utf-8 -*-
import numpy as np
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from datetime import datetime
from configparser import ConfigParser
import csv
#from sensor import Sensor
from graph import Graph


###############################################################################
# MAIN
if __name__ == '__main__':

    if len( sys.argv ) < 2:
        print ("Usage: o2mb_plot <csv_file>")
        sys.exit()

    logfile = sys.argv[1]

    #cfg     = ConfigParser()
    #cfg.read( "o2mb.ini" )

    #gcfg    = ConfigParser()
    #gcfg.read( "o2mb_graph.ini")

    #sens    = Sensor( cfg['SENSOR'] )

    fig     = plt.figure()
    fig.canvas.manager.set_window_title( logfile )

    with open( logfile, newline='' ) as csvfile:
        log     = csv.DictReader(csvfile)

        ###########################################################################
        # GRAPH INIT
        for field in log.fieldnames:
            first_column    = field
            break

        param   = []
        for k in log.fieldnames:
            if k is not first_column:
                param.append( [k, 0, 1500, 'blue', 'solid', 1,] )

        param[0][1] = 0
        #param[0][2] = 2**24
        param[0][2] = 2**22
        param[0][3] = 'orange'

        param[1][1] = 15
        param[1][2] = 45
        param[1][3] = 'red'

        param[2][1] = 950
        param[2][2] = 1050
        param[2][3] = 'green'

        param[3][1] = 4
        param[3][2] = 14
        param[3][3] = 'blue'

        '''
        ###########################################################################
        # GRAPH INIT
        param   = [
        #   name        ymin    ymax        color       linestyle   linewidth
        (   'PPM',      -50,    150,        'blue',     'dashed',   1,      ),
        (   'ADC RAW',  0,      10000000,   'orange',   'solid',    1,      ),
        (   'ADC mV',   0,      2500,       'cyan',     'solid',    1,      ),
        (   't DIGC',   15,     45,         'red',      'dashdot',  1,      ),
        (   'P hPa',    950,    1050,       'green',    'dotted',   1,      ), ]
        '''
        #xlen    = log.line_num
        xlen    = 0
        ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.90], axes_class=HostAxes )
        graph   = Graph( ax, xlen, param )



        for row in log:
            for k, v in row.items():
                if k is first_column:
                    graph.xdata.append(datetime.strptime( v, '%Y/%m/%d %H:%M:%S'))
                else:
                    graph.ydata[k].append( float(v) )

        graph.plot()

    plt.show()
