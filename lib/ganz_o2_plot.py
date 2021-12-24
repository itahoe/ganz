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
        #param[0] =  [  'adc_raw',   0,      2**24,      'orange',   'dashed',   1,          ]
        param[0] =  [  'adc_raw',   2**20,  2**22,      'orange',   'dashed',   1,          ]
        param[1] =  [  'temp_digc', 15,     45,         'red',      'dashdot',  1,          ]
        param[2] =  [  'pres_hpa',  950,    1050,       'green',    'dashdot',  1,          ]
        #param[3] =  [  'ppm_sw',    -50,    150,        'blue',     'dashed',   1,          ]
        param[3] =  [  'ppm_sw',    5,      15,         'blue',     'dashed',   1,          ]

        param.append(['ppm_sw1',    0,      10,         'magenta',  'solid',     2,] )
        param.append(['kt_cell',    0,      2,          'darkgray', 'dotted',    2,] )
        #param.append(['rmse',       0,     100,          'black',   'dotted',    1,] )


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

        #rmse    = [0.0]*100

        for i in range(log.line_num-1):
            raw = graph.ydata['adc_raw'   ][ i]
            t   = graph.ydata['temp_digc' ][ i]
            p   = graph.ydata['pres_hpa'  ][ i]
            ppm = sens.raw_to_ppm( raw, t, p )  

            graph.ydata['ppm_sw1'].append( ppm )

            kt_cell = sens.ktemp_cell_get( t )
            graph.ydata['kt_cell'].append( kt_cell )


            #rmse[1:]    = rmse[:-1]
            #rmse[0]     = val

            #rmse    = sens.rmse( graph.ydata['rmse'][ i:i-100] )
            #graph.ydata['rmse'].append( rmse )



        #print( 'adc_raw: ', len( graph.ydata['adc_raw'] ) )
        #print( '   test: ', len( graph.ydata['test']    ) )

        graph.plot()

    plt.show()
