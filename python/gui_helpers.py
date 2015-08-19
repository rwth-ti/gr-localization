#import sip
#sip.setapi('QString', 2)
#sip.setapi('QVariant', 2)

from PyQt4 import QtCore, QtGui

class receiver_item():
    def __init__(self, gain, antenna):
        self.gain = gain
        self.antenna = antenna
        self.coordinates = [0.0,0.0,0.0,0.0]

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
    def columnCount(self, parent=QtCore.QModelIndex()): return 3

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return None
        if not role==QtCore.Qt.DisplayRole: return None
        if index.column() == 0:
            return self.parent.receivers.keys()[index.row()]
        elif index.column() == 1:
            return self.parent.receivers.values()[index.row()].gain
        elif index.column() == 2:
            return self.parent.receivers.values()[index.row()].antenna

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

    def set_gain(self, gain, serial):
        print "gain:",gain
        self.parent.receivers[serial].gain = gain
        index = self.index(self.parent.receivers.keys().index(serial),1)
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
        if orientation == QtCore.Qt.Vertical:
            return "RX" + str(section + 1)

    def flags(self, index):
        if (index.column() == 0):
            return QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled
        else:
            return QtCore.Qt.ItemIsEnabled

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
