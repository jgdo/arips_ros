import os
import rospy
import rospkg
import math

from python_qt_binding import loadUi

from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import arips_arm_msgs.msg
import arips_arm_msgs.srv

class ArmGuiWidget(QWidget):
    sigMotionStateReceived = Signal(arips_arm_msgs.msg.MotionState)

    pkgpath = rospkg.RosPack().get_path('rqt_arips_arm_plugin')

    # connection state
    CSTATE_DISCONNECTED, CSTATE_CONNECTED = range(2)

    JINDEX_STATUS, JINDEX_POS, JINDEX_VEL, JINDEX_GOAL_POS, JINDEX_GOAL_VEL, JINDEX_SETPOINT = range(0, 6)

    def __init__(self, parent=None):        
        super(ArmGuiWidget, self).__init__(parent)
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(self.pkgpath, 'resource', 'ArmPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        # Give QObjects reasonable names
        self.setObjectName('ArmPluginUi')
        
        self.btnConnectDisconnect.clicked.connect(self.onConnectPressed)
        self.sigMotionStateReceived.connect(self.onMotionStateReceivedQt)

        self.connectionStatus = self.CSTATE_DISCONNECTED

        self.subMotionState = None

    def shutdown(self):
        self.disconnect()

    def onConnectPressed(self):
        if self.connectionStatus == self.CSTATE_CONNECTED:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        self.disconnect()
        self.subMotionState = rospy.Subscriber(self.prefixEdit.text() + "motion_state", arips_arm_msgs.msg.MotionState,
                                               self.onMotionStateReceived, queue_size=10)

        self.connectionStatus = self.CSTATE_CONNECTED
        self.lblStatus.setText("Connecting ...")
        self.btnConnectDisconnect.setText("Disconnect")

    def disconnect(self):
        if not self.subMotionState is None:
            self.subMotionState.unregister()
            self.subMotionState = None

        self.connectionStatus = self.CSTATE_DISCONNECTED
        self.lblStatus.setText("Disconnected")
        self.btnConnectDisconnect.setText("Connect")

    def onMotionStateReceived(self, msg):
        self.sigMotionStateReceived.emit(msg)

    def setJointItemText(self, row, col, text):
        if self.tblJointStates.item(row, col) is None:
            self.tblJointStates.setItem(row, col, QTableWidgetItem(text))
        else:
            self.tblJointStates.item(row, col).setText(text)

    def onMotionStateReceivedQt(self, msg):
        #print "onMotionStateReceivedQt"
        # print self.tblJointStates.item(0, 0).setText("bla")
        #self.tblJointStates.item(1, 0).setText("bla")
        #self.tblJointStates.item(2, 0).setText("bla")

        #self.tblJointStates.setItem(0, 1, QTableWidgetItem("Hello"))

        for i in range(0, len(msg.jointStates)):
            # print "row: " + str(i) + " col: " + str(self.JINDEX_POS)
            self.setJointItemText(i, self.JINDEX_POS, str(math.degrees(msg.jointStates[i].position)))
            self.setJointItemText(i, self.JINDEX_VEL, str(math.degrees(msg.jointStates[i].velocity)))
            self.setJointItemText(i, self.JINDEX_GOAL_POS, str(math.degrees(msg.jointStates[i].setpoint_pos)))
            self.setJointItemText(i, self.JINDEX_GOAL_VEL, str(math.degrees(msg.jointStates[i].setpoint_vel)))
