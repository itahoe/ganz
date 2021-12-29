## -*- coding: utf-8 -*-

import sys
sys.path.append('../lib')
sys.path.append('../../src/lib')

import numpy as np
import sys
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from datetime import datetime
from configparser import ConfigParser
import csv
from graph          import Graph
from sensor         import Sensor


###############################################################################
# MAIN
if __name__ == '__main__':

    if len( sys.argv ) < 2:
        print ("Usage: plot <csv_file>")
        sys.exit()

    logfile = sys.argv[1]


    conf    = ConfigParser()
    conf['DEFAULT']['config_path']  = str('ganz.ini')
    conf.read( conf['DEFAULT']['config_path'] )

    sens    = Sensor( conf )

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

        ###########################################################################
        # GRAPH INIT
        #   name                    ymin    ymax        color       linestyle   linewidth
        param[0] =  [  'adc_raw',   0,      2**24,      'orange',   'dashed',   1,          ]
        #param[0] =  [  'adc_raw',   2**20,  2**22,      'orange',   'dashed',   1,          ]
        param[1] =  [  'temp_digc', 0,      45,         'red',      'dashdot',  1,          ]
        param[2] =  [  'pres_hpa',  950,    1050,       'green',    'dashdot',  1,          ]
        param[3] =  [  'ppm_sw',    -10,    150,        'yellow',   'dotted',   1,          ]

        param.append(['ppm_kt_none', 50,    100,        'darkblue', 'dashed',   1,      ] )
        param.append(['ppm_kt_cell', 50,    100,        'blue',     'dashed',   1,      ] )
        param.append(['rms_kt_none', 0,     10,         'darkgray', 'dotted',   1,      ] )
        param.append(['rms_kt_cell', 0,     10,         'gray',     'dotted',   1,      ] )


        xlen    = 0
        ax      = fig.add_axes( [0.05, 0.05, 0.70, 0.90], axes_class=HostAxes )

        graph   = Graph( ax, xlen, param )

        for row in log:
            for k, v in row.items():
                if k is first_column:
                    graph.xdata.append(datetime.strptime( v, '%Y/%m/%d %H:%M:%S'))
                else:
                    graph.ydata[k].append( float(v) )

        print( 'offset: %f, slope: %f, ktemp: %f' % (sens.trim.offset, sens.trim.slope, sens.ktemp.ktemp) )

        rms_window_len = 600
        rms_kt_none_data    = [0.0] * rms_window_len
        rms_kt_cell_data    = [0.0] * rms_window_len


        for i in range(log.line_num-1):
            raw = graph.ydata['adc_raw'   ][ i]
            t   = graph.ydata['temp_digc' ][ i]
            p   = graph.ydata['pres_hpa'  ][ i]
            ppm = sens.raw_to_ppm( raw, t, p )
            #graph.ydata['ppm_sw'].append( ppm )

            ppm_kt_none = sens.raw_to_ppm( raw, 25.0, p )
            graph.ydata['ppm_kt_none'].append( ppm_kt_none )

            ppm_kt_cell = sens.raw_to_ppm( raw, t, p )
            graph.ydata['ppm_kt_cell'].append( ppm_kt_cell )

            rms_kt_none_data[1:]    = rms_kt_none_data[:-1]
            rms_kt_none_data[0]     = ppm_kt_none
            graph.ydata['rms_kt_none'].append( sens.rmse( rms_kt_none_data ) )

            rms_kt_cell_data[1:]    = rms_kt_cell_data[:-1]
            rms_kt_cell_data[0]     = ppm_kt_cell
            graph.ydata['rms_kt_cell'].append( sens.rmse( rms_kt_cell_data ) )


        graph.plot()

    plt.show()
