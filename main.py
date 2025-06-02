import sys, os, time, serial, threading
from datetime import datetime
import numpy as np

from PyQt6.QtCore import QSize, Qt, QTimer
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtWidgets import (
    QApplication, 
    QMainWindow,
    QWidget, 
    QPushButton,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout)

from pyqtgraph.opengl import GLViewWidget, MeshData, GLMeshItem
from pyqtgraph import GraphicsLayoutWidget, PlotWidget, ViewBox, AxisItem, PlotCurveItem, mkPen
from stl import mesh

XBEE_COM_PORT = "/dev/ttyUSB0"
SIM_DATA_FILE = "sim_data.csv"
MESH_FILE = "Container_old.stl"

SCRIPT_DIR = os.path.dirname(__file__)


class XbeeDriverSim():
    def __init__(self, gui):
        self.gui = gui

        self._msg = ""
        self._unread = False
        self.received_count = 0
        self.last_sent_command = ""

        with open(os.path.join(SCRIPT_DIR, "data", SIM_DATA_FILE), "r") as file:
            self.msgs = file.readlines()
        self.c = 0

        #simulated packet 
        timer = QTimer(gui)
        timer.timeout.connect(self.new_data)
        timer.start(1000)

    def new_data(self):
        if self._unread:
            print("Warning Data lost")

        self._msg = self.msgs[self.c][:-1]
        self._msg = self._msg.split(',')
        end_of_echo = self._msg.index("")
        self._msg = self._msg[:24] + [','.join(self._msg[24:end_of_echo])] + self._msg[end_of_echo:]

        self._msg[24] = self.last_sent_command


        self.received_count += 1
        self.c += 1
        self._unread = True

    def get_latest_msg(self):
        self._unread = False
        return self._msg


    def is_unread(self):
        return self._unread
    
    def send_cmd(self, cmd):
        print("Sending ", cmd)
        #self.gui.variables["CMD Echo"].setStatus("Warn")
        #self.gui.variables["CMD Echo Line"].setStatus("Warn")

        self.last_sent_command = cmd

class XbeeDriver():
    def __init__(self, gui, COM=XBEE_COM_PORT, BAUD=115200):

        self.filename = datetime.now().strftime("%H-%M-%S_%d-%m-%Y") + '.csv'
        #self.abs_filename = os.path.join(SCRIPT_DIR, "logs", self.filename)

        self.gui = gui
        self.ser = serial.Serial(COM, BAUD)
        self.ser.timeout = 0.01

        self.xbee_thread = threading.Thread(target=self.xbee_handler, daemon=True)
        self._xbee_lock = threading.Lock()
        self._kill_flag = False
        self.xbee_thread.start()

        self.simp_thread = threading.Thread(target=self.simp_handler, daemon=True)
        self.simp_thread.start()
        self._simp_lock = threading.Lock() #only used to check the exit flag
        self._simp_running_flag = False
        self.simp_state = False
        self._toSendSimp = ''

        self._msg = ""
        self._unread = False
        self._recv_count = 0
        self._toSend = ""
        self.last_sent_command = "-"

        print("Xbee driver: finished init")
        print(self.filename)

    def close(self):
        self.stop_simp()
        with self._xbee_lock: self._kill_flag = True
        self.ser.close()

    def xbee_handler(self): #reads from xbee and writes to xbee
        latest_msg = ''
        while True:
            try:
                with self._xbee_lock:
                    if self._kill_flag: 
                        break
                    latest_char = self.ser.read().decode()
                with open(os.path.join(SCRIPT_DIR, "logs", self.filename), 'a') as file:
                    file.write(latest_char)
        
                if latest_char == "\n":
                    with self._xbee_lock:
                        if self._unread:
                            print("xbee_handler: MSG OVERFLOW, data lost")
                        print("xbee handler: got packet:")
                        self._msg = latest_msg.split(',')
                        print(self._msg)

                        end_of_echo = self._msg.index('') #remove splits in echo (every command has commas)
                        self._msg = self._msg[:24] + [','.join(self._msg[24:end_of_echo])] + self._msg[end_of_echo:]
                        print(self._msg)
                        print(len(self._msg))
                        latest_msg = ''
                        print("xbee_handler: got new msg")
                        self._unread = True
                        self._recv_count += 1

                        if self._toSendSimp != '':
                            print("xbee handler: allowing single simp send")
                            self.ser.write(self._toSendSimp.encode())
                            self.last_sent_command = ('\n\n' + self._toSendSimp).split('\n')[-2]
                            self._toSendSimp = ''
                            print('done')
                else:
                    latest_msg += latest_char

                with self._xbee_lock:
                    if self._toSend != '':
                        print("xbee_handler sending: ", self._toSend)
                        self.ser.write(self._toSend.encode())
                        self.last_sent_command = ('\n\n' + self._toSend).split('\n')[-2]
                        self._toSend = ''
            except Exception as e:
                print("xbee handler: ERROR")
                print(str(e))


    def simp_handler(self):
        with open(os.path.join(SCRIPT_DIR, "data", SIM_DATA_FILE), "r") as file:
            all_data = file.readlines()

        while True:
            self.simp_c = 0
            self.simp_state = True
            time.sleep(0.5)
            while self.simp_state:
                with self._simp_lock:
                    self.simp_state = self._simp_running_flag  
                if self.simp_state:   
                    print("SimP handler: sending packet", self.simp_c)
                    self.send_simp_msg("CMD,3130,SIMP," + str(0.1) + "\n")
                    self.simp_c += 1

                    simp_timer = time.time()
                    while time.time() - simp_timer < 1:
                        time.sleep(0.1)
            #print("SimP handler: Terminating")

    def start_simp(self):
        print("STARTING SIMP")
        with self._simp_lock:
            self._simp_running_flag = True
            print(self._simp_running_flag)
        return 1
    
    def stop_simp(self):
        print("STOPPING SIMP")
        with self._simp_lock:
            self._simp_running_flag = False
        return 0

    def is_unread(self):
        with self._xbee_lock:
            x = self._unread
        return x
    
    def get_msg(self):
        with self._xbee_lock:
            msg = self._msg
            self._unread = False
        return msg
    
    def get_recv_count(self):
        with self._xbee_lock:
            x = self._recv_count
        return x
    
    def send_msg(self, msg):
        if "<UTC TIME>" in msg: #replace <UTC TIME> with HH:MM:SS timestamp
            print("Replacing ", msg)
            start = msg.find("<UTC TIME>")
            end = msg[start:].find(">") + start + 1
            msg = msg[:start] + datetime.now().strftime("%H:%M:%S") + msg[end:]
            print("with", msg)

        with self._xbee_lock:
                self._toSend += msg
        return True
    
    def send_simp_msg(self, msg):
        with self._xbee_lock:
                self._toSendSimp = msg
        #self.last_sent_command = msg
        return True

class Graphic3d(GraphicsLayoutWidget):
    def __init__(self, file):
        super().__init__()
        self.setBackground("w")
        stl_mesh = mesh.Mesh.from_file(os.path.join(SCRIPT_DIR, "mesh", file)) #Eiffel_tower_sample.STL
        points = stl_mesh.points.reshape(-1, 3)
        faces = np.arange(points.shape[0]).reshape(-1, 3)
        mesh_data = MeshData(vertexes=points, faces=faces)
        mesh_model = GLMeshItem(meshdata=mesh_data, smooth=True, drawFaces=False, drawEdges=True, edgeColor=(0, 0, 0, 1))

        window = GLViewWidget()
        window.setBackgroundColor("w")
        window.addItem(mesh_model)

        layout = QHBoxLayout()
        layout.addWidget(window)
        self.setLayout(layout)
        #self.setFixedWidth(700)
        self.setFixedHeight(300)

class GraphWidget(GraphicsLayoutWidget):
    def __init__(self, GUI):
        super().__init__()
        self.GUI = GUI

        self.setBackground("w")
        self.plot = PlotWidget()
        self.plot.setBackground("w")
        layout = QHBoxLayout()
        layout.setContentsMargins(0,5,0,0)
        #self.setFixedHeight(200)
        self.setContentsMargins(5,10,20,40)

        self.p0 = self.plot.plotItem

        self.line_colours = [(255, 0, 0),
                             (0, 255, 0),
                             (0, 0, 255)]
        self.fonts = []
        self.pens = []
        for i in range(len(self.line_colours)):
            self.fonts.append(QFont())
            self.fonts[i].setPointSize(14)
            self.fonts[i].setWeight(200)
            self.fonts[i].setBold(True)

            self.pens.append(mkPen(color=self.line_colours[i], width=3))
        self.pens.append(mkPen(color=(255,255,255), width=1))
        

        self.p0.getAxis('bottom').setPen('k')


        self.p1 = ViewBox()
        self.p0.showAxis('left')
        self.p0.scene().addItem(self.p1)
        self.p0.getAxis('left').linkToView(self.p1)
        self.p1.setXLink(self.p0)
        self.p0.getAxis('left').setPen('k')
        self.p0.getAxis('left').label.setFont(self.fonts[0])
        self.p0.getAxis('left').setTickFont(self.fonts[0])
        self.p0.getAxis('left').setTextPen(self.line_colours[0])
        self.p0.getAxis('left').setLabel('axis 1')

        self.p2 = ViewBox()
        self.p0.showAxis('right')
        self.p0.scene().addItem(self.p2)
        self.p0.getAxis('right').linkToView(self.p2)
        self.p2.setXLink(self.p0)
        self.p0.getAxis('right').setPen('k')
        self.p0.getAxis('right').label.setFont(self.fonts[1])
        self.p0.getAxis('right').setTickFont(self.fonts[1])
        self.p0.getAxis('right').setTextPen(self.line_colours[1])
        self.p0.getAxis('right').setLabel('axis 2')

        ## create third ViewBox. 
        ## this time we need to create a new axis as well.
        self.p3 = ViewBox()
        self.ax3 = AxisItem('right')
        self.p0.layout.addItem(self.ax3, 2, 3)
        self.p0.scene().addItem(self.p3)
        self.ax3.linkToView(self.p3)
        self.p3.setXLink(self.p0)
        self.ax3.setZValue(-10000)
        self.ax3.setPen('k')
        self.ax3.label.setFont(self.fonts[2])
        self.ax3.setTickFont(self.fonts[2])
        self.ax3.setTextPen(self.line_colours[2])
        self.ax3.setLabel('axis 3')

        self.updateViews()
        self.p0.vb.sigResized.connect(self.updateViews)
        self.p0.setDefaultPadding(0)

        self.autorange_line = self.p0.plot([-5,-4,-3,-2,-1,0], [1,2,4,8,16,32], pen=self.pens[3])

        self.line1 = PlotCurveItem([1,2,4,8,16,32], pen=self.pens[0])
        self.line2 = PlotCurveItem([10,20,40,80,40,20], pen=self.pens[1])
        self.line3 = PlotCurveItem([3200,1600,800,400,200,100], pen=self.pens[2])
        self.p1.addItem(self.line1)
        self.p2.addItem(self.line2)
        self.p3.addItem(self.line3)

        layout.addWidget(self.plot)
        self.setLayout(layout)

    def updateViews(self):
        ## view has resized; update auxiliary views to match
        self.p1.setGeometry(self.p0.vb.sceneBoundingRect())
        self.p2.setGeometry(self.p0.vb.sceneBoundingRect())
        self.p3.setGeometry(self.p0.vb.sceneBoundingRect())
        
        ## need to re-update linked axes since this was called
        ## incorrectly while views had different shapes.
        ## (probably this should be handled in ViewBox.resizeEvent)
        self.p1.linkedViewChanged(self.p0.vb, self.p1.XAxis)
        self.p2.linkedViewChanged(self.p0.vb, self.p2.XAxis)
        self.p3.linkedViewChanged(self.p0.vb, self.p3.XAxis)

    def setData(self, t, variable_1, variable_2, variable_3):

        if t == []:
            print("empty")
            return

        #self.p1.setXRange(-5,0,padding=0)
        #self.p1.setRange(xRange=(-5, 5), disableAutoRange=False)
        self.p0.getAxis("left").setLabel(self.genLabel(variable_1))
        #print(variable_1.name.text())
        self.line1.setData(t[-len(variable_1.history):], variable_1.history)

        try:
            if variable_2:
                self.p0.getAxis("right").show()
                self.p0.getAxis("right").setLabel(self.genLabel(variable_2))
                self.p2.show()
                self.line2.setData( [i[0] for i in self.variables[variable_2].history], [i[1] for i in self.variables[variable_2].history])
            else:
                self.p0.getAxis("right").hide()
                self.p2.hide()
        except:
            print("error plotting line 2")
            self.p0.getAxis("right").hide()
            self.p2.hide()

        try:
            if variable_3:
                self.ax3.show()
                self.ax3.setLabel(self.genLabel(variable_3))
                self.p3.show()
                self.line3.setData(t[-len(variable_3.history):], variable_3.history)
            else:
                self.ax3.hide()
                self.p3.hide()
        except:
            print("error plotting line 3")
            self.ax3.hide()
            self.p3.hide()


    def setDataSmart(self, variable_1, variable_2, variable_3):

        if self.GUI.variables["State"].getData() == "LAUNCH_PAD": #All Data with t=0 at this instant ~ time.time()
            if self.GUI.variables["Substate"].getData() == "Disarmed":
                max = 0
                min = self.GUI.start_time - time.time()
                t_offset = time.time()
            else:
                max = 0
                min = -10
                t_offset = time.time()


        else: #10s before launch to now
            max = time.time() - self.GUI.launch_time
            min = -10
            t_offset = self.GUI.launch_time
    

        self.autorange_line.setData([min, min+0.0000001, max], [0,1,1])


        

        try:
            if variable_1:
                self.p0.getAxis("left").show()
                self.p0.getAxis("left").setLabel(self.genLabel(self.GUI.variables[variable_1]))
                self.p1.show()
                self.line1.setData( [i[1] - t_offset for i in self.GUI.variables[variable_1].history], [i[0] for i in self.GUI.variables[variable_1].history], connect="finite")
            else:
                self.p0.getAxis("left").hide()
                self.p1.hide()
        except Exception as e:
            print("ERROR : "+str(e))
            print("error plotting line 1")
            self.p0.getAxis("left").hide()
            self.p1.hide()

        try:
            if variable_2:
                self.p0.getAxis("right").show()
                self.p0.getAxis("right").setLabel(self.genLabel(self.GUI.variables[variable_2]))
                self.p2.show()
                self.line2.setData( [i[1] - t_offset for i in self.GUI.variables[variable_2].history], [i[0] for i in self.GUI.variables[variable_2].history], connect="finite")
            else:
                self.p0.getAxis("right").hide()
                self.p2.hide()
        except:
            print("error plotting line 2")
            self.p0.getAxis("right").hide()
            self.p2.hide()

        try:
            if variable_3:
                self.ax3.show()
                self.ax3.setLabel(self.genLabel(self.GUI.variables[variable_3]))
                self.p3.show()
                self.line3.setData( [i[1] - t_offset for i in self.GUI.variables[variable_3].history], [i[0] for i in self.GUI.variables[variable_3].history], connect="finite")
            else:
                self.ax3.hide()
                self.p3.hide()
        except:
            print("error plotting line 3")
            self.ax3.hide()
            self.p3.hide()

    def genLabel(self, variable):
        label = variable.name.text()
        if variable.unit.text() != "": label += " - " + variable.unit.text()
        return label

class variable_line(QWidget):
    def __init__(self, name, unit, number_check):
        super().__init__()

        self.status_colors = {"OK":     QColor(255, 255, 255, 255),
                              "Warn":   QColor(255, 255,   0, 255),
                              "Error":  QColor(255,   0,   0, 255),
                              "Off":    QColor(100, 100, 100, 255),
                              }
        self.history = []
        self.number_check = number_check


        layout = QHBoxLayout()
        layout.setContentsMargins(5,0,5,0)

        self.name = QLabel(name.upper())
        #self.name.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.name)

        layout.addStretch()

        self.data = QLabel("NO DATA")
        self.data.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self.data)

        self.unit = QLabel(unit)
        self.unit.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.unit.setFixedWidth(35)
        layout.addWidget(self.unit)

        self.setLayout(layout)
        self.setStatus("OK")
        self.setAutoFillBackground(True)
        self.setStyleSheet("QLabel{font-size: 14pt;}")

    def setStatus(self, status):
        if status in self.status_colors:
            color = self.status_colors[status]
        else:
            color = self.status_colors["Error"]

        p = self.palette()
        p.setColor(self.backgroundRole(), color)
        self.setPalette(p)

    def setData(self, new_value):
        self.data.setText(new_value)

        try:
            self.history.append([float(new_value), time.time()])
            self.setStatus("OK")
        except ValueError: 
            self.history.append([np.nan, time.time()])
            if self.number_check:
                self.setStatus("Warn")

    def getData(self):
        return self.data.text()

class LargeLabel(QWidget):
    def __init__(self, text):
        super().__init__()

        self.status_colors = {"OK":     QColor(255, 255, 255, 255),
                              "Warn":   QColor(255, 255,   0, 255),
                              "Error":  QColor(255,   0,   0, 255),
                              "Off":    QColor(100, 100, 100, 255),
                              }

        layout = QVBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        self.data = QLabel(text.upper())
        layout.addWidget(self.data)
        self.setLayout(layout)

        self.setStatus("OK")
        self.setAutoFillBackground(True)
        self.setStyleSheet("QLabel{font-size: 25pt;}")

    def setStatus(self, status):
        if status in self.status_colors:
            color = self.status_colors[status]
        else:
            color = self.status_colors["Error"]

        p = self.palette()
        p.setColor(self.backgroundRole(), color)
        self.setPalette(p)

    def setData(self, new_value):
        self.data.setText(new_value)

    def getData(self):
        return self.data.text()

class VariableWindow(QWidget):
    def __init__(self, name, variable_widgets):
        super().__init__()

        layout = QVBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)

        #self.setFrameStyle(QFrame.Panel | QFrame.Plain)
        #QFrame.
        #self.setFrameStyle(255)
        #self.setLineWidth(1)

        self.banner = QWidget()
        self.banner.setAutoFillBackground(True)
        self.setStatus("OK")
        banner_layout = QHBoxLayout()
        banner_layout.setContentsMargins(10,0,10,0)
        banner_layout.setSpacing(0)
        

        self.name = QLabel("<b>" + name + "</b>")
        banner_layout.addStretch()
        self.name.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        banner_layout.addWidget(self.name)
        banner_layout.addStretch()

        self.state = QLabel("<b></b>")
        self.state.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight)
        banner_layout.addWidget(self.state)

        self.banner.setLayout(banner_layout)
        layout.addWidget(self.banner)
        
        for widget in variable_widgets:
            layout.addWidget(widget)

        self.setLayout(layout)
        self.setStyleSheet("QLabel{font-size: 14pt;}")
        self.setFixedWidth(300)

    def setStatus(self, status):
        self.status_colors = {"OK":     QColor(200, 200, 200, 255),
                              "Warn":   QColor(255, 255,   0, 255),
                              "Error":  QColor(255,   0,   0, 255),
                              "LOS":    QColor(255,   0,   0, 255),
                              "Off":    QColor(100, 100, 100, 255),
                              }
        
        if status in self.status_colors:
            color = self.status_colors[status]
        else:
            color = self.status_colors["Error"]

        p = self.banner.palette()
        p.setColor(self.backgroundRole(), color)
        self.banner.setPalette(p)

class MainVariableWindow(QWidget):
    def __init__(self, name, variables):
        super().__init__()

        variable_names = ["Mission Time",
                          "Altitude",
                          "Descent Rate"]

        layout = QVBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)

        state_bar = QWidget()
        layout2 = QHBoxLayout()
        layout2.setContentsMargins(0,0,0,0)
        layout2.setSpacing(0)
        layout2.addWidget(variables["State"])
        variables["State"].data.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        layout2.addWidget(variables["Mode"])
        variables["Mode"].setFixedWidth(80)
        state_bar.setLayout(layout2)


        substate_bar = QWidget()
        layout3 = QHBoxLayout()
        layout3.setContentsMargins(0,0,0,0)
        layout3.setSpacing(0)
        layout3.addWidget(variables["Substate"])
        variables["Substate"].data.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        spacer = LargeLabel("")
        layout3.addWidget(spacer)
        spacer.setFixedWidth(80)
        substate_bar.setLayout(layout3)
    
        layout.addWidget(state_bar)
        layout.addWidget(substate_bar)
        for i in variable_names:
            layout.addWidget(variables[i])
            variables[i].setStyleSheet("QLabel{font-size: 18pt;}")
            variables[i].unit.setFixedWidth(50)
        
        #layout.addStretch()

        self.setLayout(layout)
        self.setFixedWidth(600)
        #self.setObjectName("Padded")
        #self.setStyleSheet("QWidget#Padded { padding: 100px; background-color: green;}")
        
class TXButton(QPushButton):
    def __init__(self, GUI, params): # params = [Text, Command]
        super().__init__(params[0])
        self.command = params[1]
        self.GUI = GUI
        self.setFixedHeight(90)

        if self.command != "CLEAR GRAPH":
            self.clicked.connect(lambda : self.GUI.xbee_driver.send_msg(self.command))

class ButtonWindow(QWidget):
    def __init__(self, buttons):
        super().__init__()

        layout = QGridLayout()
        layout.setContentsMargins(0,0,0,0)
        for i in range(len(buttons)):
            layout.addWidget(buttons[i], i//2, i%2)

        #layout.setRowStretch(layout.rowCount(), 1)
        self.setLayout(layout)
        self.setFixedWidth(225)
        self.setStyleSheet("QPushButton{font-size: 14pt;}")






# Subclass QMainWindow to customize GCS main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.xbee_driver = XbeeDriver(self, XBEE_COM_PORT, 115200)
        self.launch_packet = -1
        self.last_msg_time = time.time()
        self.launch_time = time.time()

        self.setWindowTitle("CANSAT Ground Station")

        self.status_colors = {"LAUNCH_PAD":     QColor(226, 135,  67, 255),
                              "ASCENT":         QColor(255, 255,   0, 255),
                              "APOGEE":         QColor(255,   0,   0, 255),
                              "DESCENT":        QColor(100, 100, 100, 255),
                              "PROBE RELEASE":  QColor(100, 100, 100, 255),
                              "Error":          QColor(100,   0,   0, 255),
                              }

        self.variable_names =  [["State", ""], #Unique ID Name, Short Name, Unit, "numerical_check"
                                ["Substate", ""],
                                ["Mode", ""],

                                ["Descent Rate", "m/s", True],
                                ["Mission Time", "UTC", False],

                                ["Temperature", "°C", True],
                                ["Pressure", "kPa", True], 
                                ["Altitude", "m", True],
                                ["Altitude 2", "m", True],   
                                ["GYRO R", "°/s", True],
                                ["GYRO P", "°/s", True],
                                ["GYRO Y", "°/s", True],
                                ["ACCEL R", "°/s²", True],
                                ["ACCEL P", "°/s²", True],
                                ["ACCEL Y", "°/s²", True],
                                ["MAG R", "", True],
                                ["MAG P", "", True],
                                ["MAG Y", "", True],
                                ["GPS Time", "UTC", False],
                                ["GPS Altitude", "m", True],
                                ["GPS Lat", "°N", True],
                                ["GPS Long", "°W", True],
                                ["GPS Sats", "", True],
                                ["Bus Voltage", "V", True],
                                ["Bus Current", "A", True],
                                ["Bus Power", "W", True],
                                ["Main SOC", "%", True],

                                ["Autogyro Rate", "°/s", True],
                                ["Release Mechanism", "", False],
                                
                                ["Packet Count", "", True],
                                ["Received Count", "", True],
                                ["CMD Echo", "", False],
                                ["CMD Echo Line", "", False],

                                ["Gimbal State", "", False],
                                
                                ["CAM1 State", "", False],
                                ["CAM1 WiFi", "", False],
                                ["CAM1 Camera", "", False],
                                
                                ["CAM2 State", "", False],
                                ["CAM2 Battery Voltage", "V", True],
                                ["CAM2 WiFi", "", False],
                                ["CAM2 Camera", "", False]]
        self.variables = {}
        
        for i in self.variable_names[:3]:
            self.variables[i[0]] = LargeLabel(i[0])
        self.variables["State"].setData("Not Connected")
        self.variables["Substate"].setData("Substate x")
        self.variables["Substate"].setStyleSheet("QLabel{font-size: 20pt;}")
        self.variables["Mode"].setData("(X)")
        for i in self.variable_names[3:]:
            self.variables[i[0]] = variable_line(i[0], i[1], i[2])
            if "GPS" in i[0]:
                self.variables[i[0]].name.setText(i[0][4:].upper())
            if "CAM" in i[0]:
                self.variables[i[0]].name.setText(i[0][5:].upper())

        self.variables["Altitude 2"].name.setText("ALTITUDE")
        #self.variables["Altitude 2"].name.setText("ALTITUDE")
        self.variables["Gimbal State"].name.setText("STATE")
        self.variables["Release Mechanism"].name.setText("MECHANISM")

        self.variables["CMD Echo"].data.setText("")
        self.variables["CMD Echo Line"].name.setText("")
        #self.variables["CMD Echo line"].
        self.variables["CMD Echo Line"].data.setText("-")


        TEAM_ID = "3130"
        self.button_names =    [["Arm",                 "CMD," + TEAM_ID + ",ARM,ON\n"],
                                ["Disarm",              "CMD," + TEAM_ID + ",ARM,OFF\n"],
                                ["CX ON",               "CMD," + TEAM_ID + ",CX,ON\n"],
                                ["CX OFF",              "CMD," + TEAM_ID + ",CX,OFF\n"],
                                ["Set Time\nUTC",       "CMD," + TEAM_ID + ",ST,<UTC TIME>\n"],
                                ["Set Time\nGPS",       "CMD," + TEAM_ID + ",ST,GPS\n"],
                                ["Calibrate\nAltitude", "CMD," + TEAM_ID + ",CAL\n"],
                                ["Simulation\nDisable", "CMD," + TEAM_ID + ",SIM,DISABLE\n"],
                                ["Simulation\nEnable",  "CMD," + TEAM_ID + ",SIM,ENABLE\n"],
                                ["Simulation\nActivate","CMD," + TEAM_ID + ",SIM,ACTIVATE\n"],
                                ["Probe\nUnlatch",      "CMD," + TEAM_ID + ",MEC,SEPERATION,ON\n"],
                                ["Probe\nLatch",        "CMD," + TEAM_ID + ",MEC,SEPERATION,OFF\n"],
                                ["Gimbal\nOn",          "CMD," + TEAM_ID + ",MEC,GIMBAL,ON\n"],
                                ["Gimbal\nOff",         "CMD," + TEAM_ID + ",MEC,GIMBAL,OFF\n"],
                                ["Clear\n Graph", "CLEAR GRAPH"]]
        self.only_on_ground =   ["Arm", "CX OFF", "Set Time\nUTC", "Set Time\nGPS", "Calibrate\nAltitude", "Simulation\nEnable", "Simulation\nActivate"]
        self.buttons = {}
        for i in self.button_names:
            self.buttons[i[0]] = TXButton(self, i)

        self.buttons["Simulation\nActivate"].clicked.connect(self.xbee_driver.start_simp)
        self.buttons["Simulation\nDisable"].clicked.connect(self.xbee_driver.stop_simp)
        self.buttons["Clear\n Graph"].clicked.connect(self.clear_graph)

        comms_data = [self.variables["Packet Count"],
                      self.variables["Received Count"],
                      self.variables["CMD Echo"],
                      self.variables["CMD Echo Line"]]
        self.comms_window = VariableWindow("Comms", comms_data)
        for i in comms_data:
            i.unit.setFixedWidth(0)

        baro_window = VariableWindow("Barometer",   [self.variables["Pressure"],
                                                     self.variables["Temperature"],
                                                     self.variables["Altitude 2"]])
        #baro_window.setStatus("LOS")
        #baro_window.state.setText("LOS 01:23")

        gps_window = VariableWindow("GPS", [self.variables["GPS Time"],
                                            self.variables["GPS Altitude"],
                                            self.variables["GPS Lat"],
                                            self.variables["GPS Long"],
                                            self.variables["GPS Sats"]])

        imu_window = VariableWindow("IMU", [self.variables["GYRO R"],
                                            self.variables["GYRO P"],
                                            self.variables["GYRO Y"],
                                            self.variables["ACCEL R"],
                                            self.variables["ACCEL P"],
                                            self.variables["ACCEL Y"],
                                            self.variables["MAG R"],
                                            self.variables["MAG P"],
                                            self.variables["MAG Y"]])

        power_window = VariableWindow("Power", [self.variables["Main SOC"],
                                                self.variables["Bus Voltage"],
                                                self.variables["Bus Current"],
                                                self.variables["Bus Power"]])

        seperartion_window = VariableWindow("Seperation", [self.variables["Release Mechanism"],
                                                           self.variables["Autogyro Rate"]])

        gimbal_window = VariableWindow("Gimbal", [self.variables["Gimbal State"]])

        cam1_window = VariableWindow("Cam 1", [self.variables["CAM1 State"],
                                               self.variables["CAM1 WiFi"],
                                               self.variables["CAM1 Camera"],])
        
        cam2_window = VariableWindow("Cam 2", [self.variables["CAM2 State"],
                                               self.variables["CAM2 Battery Voltage"],
                                               self.variables["CAM2 WiFi"],
                                               self.variables["CAM2 Camera"],])

        big_layout = QHBoxLayout()

        layout = QVBoxLayout()
        layout.setContentsMargins(10,0,5,0)
        layout.setSpacing(5)

        #layout.addStretch()
        layout.addWidget(MainVariableWindow("Main", self.variables))
        #layout.addStretch()

        variable_panels = QWidget()
        variable_panels_layout = QHBoxLayout()
        variable_panels_layout.setContentsMargins(0,0,0,0)
        variable_panels_layout.setSpacing(5)

        variable_panel_1 = QWidget()
        variable_panel_1_layout = QVBoxLayout()
        variable_panel_1_layout.setContentsMargins(0,0,0,0)
        variable_panel_1_layout.setSpacing(5)
        variable_panel_1_layout.addStretch()
        
        variable_panel_1_layout.addWidget(self.comms_window)
        variable_panel_1_layout.addWidget(baro_window)
        variable_panel_1_layout.addWidget(seperartion_window)
        variable_panel_1_layout.addWidget(gimbal_window)
        variable_panel_1_layout.addWidget(cam1_window)
        variable_panel_1_layout.addWidget(cam2_window)
        variable_panel_1.setLayout(variable_panel_1_layout)

        variable_panel_2 = QWidget()
        variable_panel_2_layout = QVBoxLayout()
        variable_panel_2_layout.setContentsMargins(0,0,0,0)
        variable_panel_2_layout.setSpacing(5)

        variable_panel_2_layout.addWidget(power_window)
        variable_panel_2_layout.addWidget(gps_window)
        variable_panel_2_layout.addWidget(imu_window)

        #variable_panel_2_layout.addWidget(ButtonWindow())
        #variable_panel_2_layout.addStretch()
        variable_panel_2.setLayout(variable_panel_2_layout)

        variable_panels_layout.addWidget(variable_panel_1)
        variable_panels_layout.addWidget(variable_panel_2)
        #variable_panels_layout.addWidget(ButtonWindow())

        variable_panels.setLayout(variable_panels_layout)
        variable_panels.setFixedWidth(600)

        layout.addWidget(variable_panels)
        left_panel = QWidget()
        left_panel.setLayout(layout)
        left_panel.setFixedWidth(610)

        graph_panel = QWidget()
        graph_panel_layout = QVBoxLayout()
        graph_panel_layout.setContentsMargins(0,0,0,0)

        self.graph3d = Graphic3d(MESH_FILE)
        graph_panel_layout.addWidget(self.graph3d)
        self.graph_1 = GraphWidget(self)
        graph_panel_layout.addWidget(self.graph_1)
        self.graph_2 = GraphWidget(self)
        graph_panel_layout.addWidget(self.graph_2)
        graph_panel.setLayout(graph_panel_layout)

        big_layout.addWidget(left_panel)
        big_layout.addWidget(graph_panel)
        #big_layout.addStretch()

        all_buttons = []
        for i in self.buttons:
            all_buttons.append(self.buttons[i])
        big_layout.addWidget(ButtonWindow(all_buttons))


        

        #self.setFixedSize(QSize(1500, 780))
        #self.setMaximumSize(1000,8000)
        widget = QWidget()
        widget.setLayout(big_layout)
        #left_panel.setStyleSheet("QWidget{background-color: blue;}")
        self.setAutoFillBackground(True)
        self.setStatus("LAUNCH_PAD")
        #widget.setFixedWidth(300)
        # Set the central widget of the Window.
        self.setCentralWidget(widget)


        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(10)
        self.c = 0
        self.start_time = -1

    def setStatus(self, status):
        if status in self.status_colors:
            color = self.status_colors[status]
        else:
            color = self.status_colors["Error"]

        p = self.palette()
        p.setColor(self.backgroundRole(), color)
        self.setPalette(p)

    def clear_graph(self):
        for var in self.variables:
            self.variables[var].history = []
        self.start_time = time.time()

    def update(self):

        if self.start_time == -1: self.start_time = time.time()

        # disable buttons while flying
        #for i in self.only_on_ground:
         #   self.buttons[i].setEnabled(self.variables["State"].getData() == "LAUNCH_PAD")

        if self.variables["CMD Echo Line"].getData() == self.xbee_driver.last_sent_command:
            self.variables["CMD Echo"].setStatus("OK")
            self.variables["CMD Echo Line"].setStatus("OK")
        else:
            #print("expecting:")
            #print(self.xbee_driver.last_sent_command)
            #print("got")
            #print(self.variables["CMD Echo Line"].getData())
            self.variables["CMD Echo"].setStatus("Warn")
            self.variables["CMD Echo Line"].setStatus("Warn")


        # LOS detector
        if self.variables["State"] != "LAUNCH_PAD" or True:
            if time.time() - self.last_msg_time > 1.20:
                los_time = time.time() - self.last_msg_time
                self.comms_window.setStatus("Error")
                los_time_str = time.strftime("%M:%S", time.gmtime(los_time))
                self.comms_window.state.setText("<b>LOS " + los_time_str + "<b>")



        if self.xbee_driver.is_unread():
            new_msg = self.xbee_driver.get_msg()
            self.last_msg_time = time.time()
            print(new_msg)

            if len(new_msg) != 31: #expects 32 entries (blank after command included)
                print("MALFORMED PACKET: expected 30 entries, got " + str(len(new_msg)+1))
                print(new_msg)
                self.comms_window.setStatus("Warn")
                self.comms_window.state.setText("MAL")
            else:
                self.data = new_msg
                self.comms_window.setStatus("OK")
                self.comms_window.state.setText("")

                if self.variables["State"].getData() == "LAUNCH_PAD": #keeps t0 in the future
                    self.launch_packet = int(self.data[2])
                    self.launch_time = time.time()

                self.variables["Mission Time"].setData(self.data[1])
                self.variables["Packet Count"].setData(self.data[2])
                self.variables["Received Count"].setData(str(self.xbee_driver.get_recv_count()))
                self.variables["Mode"].setData(self.data[3])
                self.variables["State"].setData(self.data[4])
                self.variables["Altitude"].setData(self.data[5])
                self.variables["Altitude 2"].setData(self.data[5])
                self.variables["Temperature"].setData(self.data[6])
                self.variables["Pressure"].setData(self.data[7])
                self.variables["Bus Voltage"].setData(self.data[8])

                self.variables["GYRO R"].setData(self.data[9])
                self.variables["GYRO P"].setData(self.data[10])
                self.variables["GYRO Y"].setData(self.data[11])
                self.variables["ACCEL R"].setData(self.data[12])
                self.variables["ACCEL P"].setData(self.data[13])
                self.variables["ACCEL Y"].setData(self.data[14])
                self.variables["MAG R"].setData(self.data[15])
                self.variables["MAG P"].setData(self.data[16])
                self.variables["MAG Y"].setData(self.data[17])

                self.variables["Autogyro Rate"].setData(self.data[18])

                self.variables["GPS Time"].setData(self.data[19])
                self.variables["GPS Altitude"].setData(self.data[20])
                self.variables["GPS Lat"].setData(self.data[21])
                self.variables["GPS Long"].setData(self.data[22])
                self.variables["GPS Sats"].setData(self.data[23])
                                                
                self.variables["CMD Echo Line"].setData(self.data[24])

                self.variables["Substate"].setData(self.data[26])
                #self.variables["Descent Rate"].setData(self.data[27])
                self.variables["Main SOC"].setData(self.data[27])
                self.variables["Bus Current"].setData(self.data[28])
                self.variables["Bus Power"].setData(self.data[29])
                self.variables["Release Mechanism"].setData(self.data[30])
            





        if hasattr(self, 'data'):
            t = [i for i in range(int(self.data[2])-self.launch_packet-len(self.variables["Altitude"].history), int(self.data[2])-self.launch_packet)]
            #self.graph_1.setData(t, self.variables["Altitude"], self.variables["Pressure"], None)
            self.graph_1.setDataSmart("Altitude", "Pressure", None)
            #self.graph_1.setDataSmart("GYRO R", "GYRO P", "GYRO Y")
            #self.graph_1.setDataSmart(None, "Pressure", None)
            #self.graph_2.setDataSmart("Autogyro Rate", "Descent Rate", "Temperature")
            self.graph_2.setDataSmart("Temperature", None, None)
            #self.graph_2.setDataSmart("ACCEL R", "ACCEL P", "ACCEL Y")




        


if __name__ == "__main__":
    print("### CANSAT Ground Station ###")
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()

    window.xbee_driver.close()
    print("bye")
