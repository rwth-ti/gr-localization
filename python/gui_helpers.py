#import sip
#sip.setapi('QString', 2)
#sip.setapi('QVariant', 2)

from PyQt4 import QtCore, QtGui
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar

class receiver_item():
    def __init__(self, gain, antenna, gain_calibration):
        self.gain = gain
        self.gain_calibration = gain_calibration
        self.antenna = antenna
        self.selected_position= "manual"
        self.coordinates = [0.0,0.0]
        self.coordinates_gps = [0.0,0.0]

class TableModelReceiversPosition(QtCore.QAbstractTableModel):
    def __init__(self, parent=None, *args):
        self.parent = parent
        QtCore.QAbstractTableModel.__init__(self, parent, *args)

    def rowCount(self, parent=QtCore.QModelIndex()): return len(self.parent.receivers)
    def columnCount(self, parent=QtCore.QModelIndex()): return 3

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return None
        if not role==QtCore.Qt.DisplayRole: return None
        if index.column() == 0:
            return self.parent.receivers.keys()[index.row()]
        if index.column() == 1:
            return self.parent.receivers.values()[index.row()].selected_position

    def setData(self, index, value, role=QtCore.Qt.DisplayRole):
        if index.column() == 1:
            str_from_index = {0 : "manual",
                              1 : "GPS",}[value]
            serial_index = self.index(index.row(),index.column()-1)
            serial = self.data(serial_index)
            self.parent.receivers[serial].selected_position = str_from_index
            self.parent.set_selected_position(str_from_index, serial)

    def set_selected_position(self, selected_position, serial):
        print "selected position:", selected_position
        self.parent.receivers[serial].selected_position = selected_position
        index = self.index(self.parent.receivers.keys().index(serial),1)
        self.dataChanged.emit(index, index)

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if (role != QtCore.Qt.DisplayRole):
            return
        if orientation == QtCore.Qt.Horizontal:
            if section == 0:
                return "Serial"
            if section == 1:
                return "Position"
            else:
                return ""
        if orientation == QtCore.Qt.Vertical:
            return "RX" + str(section + 1)

    def flags(self, index):
        if (index.column() == 0):
            return QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled
        else:
            return QtCore.Qt.ItemIsEnabled

class TableModelGuis(QtCore.QAbstractTableModel):
    def __init__(self, parent=None, *args):
        self.parent = parent
        QtCore.QAbstractTableModel.__init__(self, parent, *args)
        self.guis = []

    def rowCount(self, parent=QtCore.QModelIndex()): return len(self.guis)
    def columnCount(self, parent=QtCore.QModelIndex()): return 1

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return None
        if not role==QtCore.Qt.DisplayRole: return None
        return self.guis[index.row()]

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if (role != QtCore.Qt.DisplayRole):
            return
        if orientation == QtCore.Qt.Horizontal:
            if section == 0:
                return "Name"
        if orientation == QtCore.Qt.Vertical:
            return "GUI" + str(section + 1)

    def registerGui(self, serial):
        if not serial in self.guis:
            self.guis.append(serial)
            self.rowsInserted.emit(QtCore.QModelIndex(),0,0)

    def flags(self, index):
        if (index.column() == 0):
            return QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled
        else:
            return QtCore.Qt.ItemIsEnabled

class TableModelReceivers(QtCore.QAbstractTableModel):
    def __init__(self, parent=None, *args):
        self.parent = parent
        QtCore.QAbstractTableModel.__init__(self, parent, *args)

    def rowCount(self, parent=QtCore.QModelIndex()): return len(self.parent.receivers)
    def columnCount(self, parent=QtCore.QModelIndex()): return 4

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return None
        if not role==QtCore.Qt.DisplayRole: return None
        if index.column() == 0:
            return self.parent.receivers.keys()[index.row()]
        elif index.column() == 1:
            return self.parent.receivers.values()[index.row()].gain
        elif index.column() == 2:
            return self.parent.receivers.values()[index.row()].antenna
        elif index.column() == 3:
            return self.parent.receivers.values()[index.row()].gain_calibration

    def setData(self, index, value, role=QtCore.Qt.DisplayRole):
        if index.column() == 1:
            serial_index = self.index(index.row(),index.column()-1)
            serial = self.data(serial_index)
            self.parent.receivers[serial].gain = value
            self.parent.set_gain(value, serial)
        if index.column() == 2:
            str_from_index = {0 : "TX/RX",
                              1 : "RX2",}[value]
            serial_index = self.index(index.row(),index.column()-2)
            serial = self.data(serial_index)
            self.parent.receivers[serial].antenna = str_from_index
            self.parent.set_antenna(str_from_index, serial)
        if index.column() == 3:
            serial_index = self.index(index.row(),index.column()-3)
            serial = self.data(serial_index)
            self.parent.receivers[serial].gain_calibration = value
            self.parent.set_gain_calibration(value, serial)

    def set_gain(self, gain, serial):
        print "gain:",gain
        self.parent.receivers[serial].gain = gain
        index = self.index(self.parent.receivers.keys().index(serial),1)
        self.dataChanged.emit(index, index)

    def set_gain_calibration(self, gain, serial):
        print "gain_calibration:",gain
        self.parent.receivers[serial].gain_calibration = gain
        index = self.index(self.parent.receivers.keys().index(serial),3)
        self.dataChanged.emit(index, index)

    def set_antenna(self, antenna, serial):
        print "antenna:",antenna
        self.parent.receivers[serial].antenna = antenna
        index = self.index(self.parent.receivers.keys().index(serial),2)
        self.dataChanged.emit(index, index)

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if (role != QtCore.Qt.DisplayRole):
            return
        if orientation == QtCore.Qt.Horizontal:
            if section == 0:
                return "Serial"
            elif section == 1:
                return "Gain"
            elif section == 2:
                return "Antenna"
            elif section == 3:
                return "Gain Calibration"
        if orientation == QtCore.Qt.Vertical:
            return "RX" + str(section + 1)

    def flags(self, index):
        if (index.column() == 0):
            return QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled
        else:
            return QtCore.Qt.ItemIsEnabled

class GpsComboDelegate(QtGui.QItemDelegate):
    def __init__(self, parent):
        QtGui.QItemDelegate.__init__(self, parent)

    def createEditor(self, parent, option, index):
        combo = QtGui.QComboBox(parent)
        li = []
        li.append("manual")
        li.append("GPS")
        combo.addItems(li)
        self.connect(combo, QtCore.SIGNAL("currentIndexChanged(int)"), self, QtCore.SLOT("currentIndexChanged()"))
        return combo

    def setEditorData(self, editor, index):
        editor.blockSignals(True)
        n_index = {"manual": 0,
                    "GPS" : 1,}[index.model().data(index)]
        editor.setCurrentIndex(n_index)
        editor.blockSignals(False)

    def setModelData(self, editor, model, index):
        model.setData(index, editor.currentIndex())

    @QtCore.pyqtSlot()
    def currentIndexChanged(self):
        self.commitData.emit(self.sender())

class ComboDelegate(QtGui.QItemDelegate):
    def __init__(self, parent):
        QtGui.QItemDelegate.__init__(self, parent)

    def createEditor(self, parent, option, index):
        combo = QtGui.QComboBox(parent)
        li = []
        li.append("TX/RX")
        li.append("RX2")
        combo.addItems(li)
        self.connect(combo, QtCore.SIGNAL("currentIndexChanged(int)"), self, QtCore.SLOT("currentIndexChanged()"))
        return combo

    def setEditorData(self, editor, index):
        editor.blockSignals(True)
        n_index = {"TX/RX": 0,
                    "RX2" : 1,}[index.model().data(index)]
        editor.setCurrentIndex(n_index)
        editor.blockSignals(False)

    def setModelData(self, editor, model, index):
        model.setData(index, editor.currentIndex())

    @QtCore.pyqtSlot()
    def currentIndexChanged(self):
        self.commitData.emit(self.sender())

class SpinBoxDelegate(QtGui.QItemDelegate):
    def __init__(self, parent):
        QtGui.QItemDelegate.__init__(self, parent)

    def createEditor(self, parent, option, index):
        editor = QtGui.QDoubleSpinBox(parent)
        editor.setMinimum(0)
        editor.setMaximum(100)
        return editor

    def setEditorData(self, spinBox, index):
        value = index.model().data(index)
        spinBox.setValue(value)

    def setModelData(self, spinBox, model, index):
        spinBox.interpretText()
        value = spinBox.value()
        model.setData(index, value)

class PushButtonPositionDelegate(QtGui.QItemDelegate):
    def __init__(self, parent):
        QtGui.QItemDelegate.__init__(self, parent)
        self.parent = parent

    def createEditor(self, parent, option, index):
        button = QtGui.QPushButton("Set position", parent)
        button.index = index
        self.connect(button, QtCore.SIGNAL("clicked()"), self, QtCore.SLOT("clicked()"))
        return button

    @QtCore.pyqtSlot()
    def clicked(self):
        # select receiver for which the operation in gui is performed
        self.parent.setting_pos_receiver = self.parent.receivers.keys()[self.sender().index.row()]
        self.parent.position_dialog.show()
        #print self.parent.setting_pos_receiver

class NavigationToolbar(NavigationToolbar):
    # only display the buttons we need
    toolitems = [t for t in NavigationToolbar.toolitems if
                 t[0] == 'Save']
                 #t[0] in ('Pan', 'Zoom', 'Save')]

class ZoomPan:
    def __init__(self):
        self.enabled = True
        self.press = None
        self.cur_xlim = None
        self.cur_ylim = None
        self.x0 = None
        self.y0 = None
        self.x1 = None
        self.y1 = None
        self.xpress = None
        self.ypress = None


    def zoom_factory(self, ax, base_scale = 2.):
        def zoom(event):
            if not self.enabled:return
            cur_xlim = ax.get_xlim()
            cur_ylim = ax.get_ylim()

            xdata = event.xdata # get event x location
            ydata = event.ydata # get event y location

            if event.button == 'up':
                # deal with zoom in
                scale_factor = 1 / base_scale
            elif event.button == 'down':
                # deal with zoom out
                scale_factor = base_scale
            else:
                # deal with something that should never happen
                scale_factor = 1

            new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
            new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor

            relx = (cur_xlim[1] - xdata)/(cur_xlim[1] - cur_xlim[0])
            rely = (cur_ylim[1] - ydata)/(cur_ylim[1] - cur_ylim[0])

            ax.set_xlim([xdata - new_width * (1-relx), xdata + new_width * (relx)])
            ax.set_ylim([ydata - new_height * (1-rely), ydata + new_height * (rely)])
            ax.figure.canvas.draw()

        fig = ax.get_figure() # get the figure of interest
        fig.canvas.mpl_connect('scroll_event', zoom)

        return zoom

    def pan_factory(self, ax):
        def onPress(event):
            if not self.enabled:return
            if event.inaxes != ax: return
            self.cur_xlim = ax.get_xlim()
            self.cur_ylim = ax.get_ylim()
            self.press = self.x0, self.y0, event.xdata, event.ydata
            self.x0, self.y0, self.xpress, self.ypress = self.press

        def onRelease(event):
            if not self.enabled:return
            self.press = None
            ax.figure.canvas.draw()

        def onMotion(event):
            if not self.enabled:return
            if self.press is None: return
            if event.inaxes != ax: return
            dx = event.xdata - self.xpress
            dy = event.ydata - self.ypress
            self.cur_xlim -= dx
            self.cur_ylim -= dy
            ax.set_xlim(self.cur_xlim)
            ax.set_ylim(self.cur_ylim)

            ax.figure.canvas.draw()

        fig = ax.get_figure() # get the figure of interest

        # attach the call back
        fig.canvas.mpl_connect('button_press_event',onPress)
        fig.canvas.mpl_connect('button_release_event',onRelease)
        fig.canvas.mpl_connect('motion_notify_event',onMotion)

        #return the function
        return onMotion
