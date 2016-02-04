#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append("../python")
import gui_helpers
from pyproj import Proj, transform
from PIL import Image
import math
import requests
from StringIO import StringIO
from mpl_toolkits.basemap import Basemap
import time

class parser():
    def __init__(self,bbox):

        # map configuration
        self.figure = plt.figure(figsize=(16,10))

        self.init_map(bbox)

    def init_map(self,bbox):

        inProj = Proj(init='epsg:4326')
        outProj = Proj(init='epsg:3857')
        x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
        x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
        x = x1-x0
        y = y1-y0
        scale = math.ceil(math.sqrt(abs(x*y/0.3136))) * 2

        r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)

        if r.status_code == 200:
            img = Image.open(StringIO(r.content))

        #img = Image.open("../maps/ict_cubes.png")

        self.ax = self.figure.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)

        #
        # create basemap
        #

        # get reference UTM grid
        lon = bbox[0]
        lat = bbox[1]
        if lat>=72:
            lat_0 = 72
            if 0<=lon and lon<= 9:
                lon_0 = 0
            elif 9<=lon and lon<= 21:
                lon_0 = 9
            elif 21<=lon and lon<= 33:
                lon_0 = 21
            elif 33<=lon and lon<= 42:
                lon_0 = 33
            else:
                lon_0 = int(lon/6)*6

        elif 56<=lat and lat<= 64:
            lat_0 = 56
            if 3<=lon and lon<=12:
                lon_0 = 3
            else:
                lon_0 = int(lon/6)*6

        else:
            lat_0 = int(lat/8)*8
            lon_0 = int(lon/6)*6

        self.basemap = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                      urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                      projection='tmerc', ax=self.ax, lon_0=lon_0, lat_0=lat_0)

        self.basemap.imshow(img, interpolation='lanczos', origin='upper')

        #self.zp = gui_helpers.ZoomPan()
        #figZoom = self.zp.zoom_factory(self.ax, base_scale = 1.5)
        #figPan = self.zp.pan_factory(self.ax)

        self.figure.tight_layout(pad=0)
        #self.figure.patch.set_visible(False)
        self.ax.axis('off')
        plt.show(block=False)

    def make_tikz_plot_delays(self, delay, correction, filename):
        filename = filename.split(".")[0] + ".tex"
        f = open(filename,"w")

        f.write("\\documentclass[conference]{IEEEtran}\n\\usepackage{graphicx}\n\\usepackage{amssymb, amsmath}\n\\usepackage[hyphens]{url}\n\\usepackage{pgfplots}\n%\\pgfplotsset{compat=1.10}\n\\begin{document}\n\\begin{figure}\n\\small\n\\centering\n\\newlength\\figureheight\n\\newlength\\figurewidth\n\\setlength\\figureheight{6.8cm}\n\\setlength\\figurewidth{0.9\\columnwidth}\n% defining custom colors\n\\definecolor{mycolor1}{rgb}{0.00000,0.75000,0.75000}%\n\\definecolor{mycolor2}{rgb}{0.75000,0.00000,0.75000}%\n\\definecolor{mycolor3}{rgb}{0.75000,0.75000,0.00000}%\n%\n\\begin{tikzpicture}\n\\begin{axis}[%\nwidth=\\figurewidth,\nheight=\\figureheight,\nscale only axis,\n%xmin=3,\n%xmax=8,\n%xtick={3, 4, 5, 6, 7, 8},\nxlabel={Acquisitions},\nxmajorgrids,\n%ymin=-14,\n%ymax=14,\nyminorticks=true,\nylabel={$\\Delta\\tau$[samples]},\ny label style={at={(0.07,0.5)}},\nymajorgrids,\nyminorgrids,\nlegend style={draw=black,fill=white,legend cell align=left}\n]\n")

        plot_1 = ""
        plot_2 = ""
        plot_3 = ""
        plot_4 = ""

        for i in range(1,len(delay)):
            plot_1 += str(i) + "\t" + str(int(delay[i-1][0])) + "\\\\\n"
            plot_2 += str(i) + "\t" + str(int(correction[i-1][0])) + "\\\\\n"
            if correction is not None:
                plot_3 += str(i) + "\t" + str(int(delay[i-1][1])) + "\\\\\n"
                plot_4 += str(i) + "\t" + str(int(correction[i-1][1])) + "\\\\\n"

        f.write("\\addplot [color=blue]\ntable[row sep=crcr]{%\n")
        f.write(plot_1)
        f.write("};\n\\addlegendentry{$\\Delta\\tau_{21}$(corrected)};\n")
        f.write("\\addplot [color=red]\ntable[row sep=crcr]{%\n")
        f.write(plot_2)
        f.write("};\n\\addlegendentry{$\\Delta\\tau_{21}$};\n")
        if correction is not None:
            f.write("\\addplot [color=green]\ntable[row sep=crcr]{%\n")
            f.write(plot_3)
            f.write("};\n\\addlegendentry{$\\Delta\\tau_{31}$(corrected)};\n")
            f.write("\\addplot [color=black]\ntable[row sep=crcr]{%\n")
            f.write(plot_4)
            f.write("};\n\\addlegendentry{$\\Delta\\tau_{31}$};\n")

        f.write("\n\\end{axis}\n\\end{tikzpicture}%\n\\end{figure}\n\\end{document}")
        f.close()



###############################################################################
# Main
###############################################################################
if __name__ == "__main__":

    f = open(sys.argv[1],"r")
    f.readline()
    f.readline()
    f.readline()
    chan_x = []
    chan_y = []
    grid_x = []
    grid_y = []
    delays_list = []
    delays_calibration_list = []
    delays_auto_calibration_list = []
    for line in f:
        adquisition = eval(eval(line))
        timestamp = adquisition[0]
        delays = adquisition[1]
        delays_calibration = adquisition[2]
        delays_auto_calibration = adquisition[3]
        sampling_rate = adquisition[4]
        frequency = adquisition[5]
        frequency_calibration = adquisition[6]
        calibration_position = adquisition[7]
        interpolation = adquisition[8]
        bandwidth = adquisition[9]
        samples_to_receive = adquisition[10]
        lo_offset = adquisition[11]
        bbox = adquisition[12]
        ref_receiver = adquisition[13]
        receivers_positions = adquisition[14]
        selected_positions = adquisition[15]
        receivers_gps = adquisition[16]
        receivers_antenna = adquisition[17]
        receivers_gain = adquisition[18]
        estimated_positions = adquisition[19]
        chan_x.append(estimated_positions["chan"]["coordinates"][0])
        chan_y.append(estimated_positions["chan"]["coordinates"][1])
        grid_x.append(estimated_positions["grid_based"]["coordinates"][0])
        grid_y.append(estimated_positions["grid_based"]["coordinates"][1])
        delays_list.append(delays)
        delays_calibration_list.append(delays_calibration)
        delays_auto_calibration_list.append(delays_auto_calibration)
    f.close()

    p = parser(bbox)

    i = 1
    for rx in receivers_positions:
        p.ax.scatter(rx[0], rx[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
        # set annotation RXx
        text = "RX" + str(i)
        i += 1
        p.ax.annotate(text, rx,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9))

    p.figure.canvas.draw()

    #for i in range(0,len(chan_x)):
    #    p.ax.scatter(chan_x[i],chan_y[i],color="blue",marker="x")
    #    p.ax.scatter(grid_x[i],grid_y[i],color="red",marker="x")
    #    time.sleep(0.1)
    #    p.figure.canvas.draw()

    p.ax.scatter(chan_x,chan_y,color="blue",marker="x")
    p.ax.scatter(grid_x,grid_y,color="red",marker="x")

    # the histogram of the data
    #n, bins, patches = plt.hist(delays_list, 50, facecolor='green', alpha=0.75)

    #delays_calibrated = np.array(delays_list)
    #delays_not_calibrated = np.array(delays_list) - np.array(delays_auto_calibration_list) - np.array(delays_calibration_list)

    #p.make_tikz_plot_delays(delays_calibrated, delays_not_calibrated, sys.argv[1])
    #plt.plot(delays_calibrated)
    #plt.plot(delays_not_calibrated)
    plt.show()
