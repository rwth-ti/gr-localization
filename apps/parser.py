#!/usr/bin/env python

import matplotlib.pyplot as plt
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
    delay = []
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
        receivers_positions = adquisition[13]
        selected_positions = adquisition[14]
        receivers_gps = adquisition[15]
        receivers_antenna = adquisition[16]
        receivers_gain = adquisition[17]
        estimated_positions = adquisition[18]
        chan_x.append(estimated_positions["chan"]["coordinates"][0])
        chan_y.append(estimated_positions["chan"]["coordinates"][1])
        grid_x.append(estimated_positions["grid_based"]["coordinates"][0])
        grid_y.append(estimated_positions["grid_based"]["coordinates"][1])
        delay.append(delays[0])
        #delay.append(delays[0]-delays_auto_calibration[0])
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
    #n, bins, patches = plt.hist(delay, 50, facecolor='green', alpha=0.75)

    plt.show()
