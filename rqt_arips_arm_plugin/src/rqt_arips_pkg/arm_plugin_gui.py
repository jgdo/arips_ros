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
        self.btnModeRawMotors.clicked.connect(self.onModeRawPressed)
        self.btnSendSetpoint.clicked.connect(self.onSendSetpointsPressed)
        self.btnModeController.clicked.connect(self.onModeControllerPressed)
        self.sigMotionStateReceived.connect(self.onMotionStateReceivedQt)

        self.connectionStatus = self.CSTATE_DISCONNECTED

        self.subMotionState = None
        self.pubMotionCommand = None
        self.pubRawMotor = None

    def shutdown(self):
        self.disconnect()

    def onConnectPressed(self):
        if self.connectionStatus == self.CSTATE_CONNECTED:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        self.disconnect()

        currentTopicRoot = self.prefixEdit.text()

        self.subMotionState = rospy.Subscriber(currentTopicRoot + "/motion_state", arips_arm_msgs.msg.MotionState,
                                               self.onMotionStateReceived, queue_size=10)
        self.pubMotionCommand = rospy.Publisher(currentTopicRoot + "/motion_command", arips_arm_msgs.msg.MotionCommand,
                                              queue_size=10)
        self.pubRawMotor = rospy.Publisher(currentTopicRoot + "/raw_motor_command", arips_arm_msgs.msg.RawMotorCommand,
                                                queue_size=10)

        self.connectionStatus = self.CSTATE_CONNECTED
        self.lblStatus.setText("Connecting ...")
        self.btnConnectDisconnect.setText("Disconnect")

    def disconnect(self):
        if self.isConnected():
            self.subMotionState.unregister()
            self.subMotionState = None
            self.pubMotionCommand.unregister()
            self.pubMotionCommand = None
            self.pubRawMotor.unregister()
            self.pubRawMotor = None

        self.connectionStatus = self.CSTATE_DISCONNECTED
        self.lblStatus.setText("Disconnected")
        self.btnConnectDisconnect.setText("Connect")

    def isConnected(self):
        return self.connectionStatus == self.CSTATE_CONNECTED

    def onMotionStateReceived(self, msg):
        self.sigMotionStateReceived.emit(msg)

    def setJointItemText(self, row, col, text):
        if self.tblJointStates.item(row, col) is None:
            self.tblJointStates.setItem(row, col, QTableWidgetItem(text))
        else:
            self.tblJointStates.item(row, col).setText(text)

    def onMotionStateReceivedQt(self, msg):
        self.lblStatus.setText(self.statusText(msg.mode))

        for i in range(0, len(msg.jointStates)):
            # print "row: " + str(i) + " col: " + str(self.JINDEX_POS)
            self.setJointItemText(i, self.JINDEX_POS, str(math.degrees(msg.jointStates[i].position)))
            self.setJointItemText(i, self.JINDEX_VEL, str(math.degrees(msg.jointStates[i].velocity)))
            self.setJointItemText(i, self.JINDEX_GOAL_POS, str(math.degrees(msg.jointStates[i].setpoint_pos)))
            self.setJointItemText(i, self.JINDEX_GOAL_VEL, str(math.degrees(msg.jointStates[i].setpoint_vel)))

    def statusText(self, mode):
        stateDict = {
            arips_arm_msgs.msg.MotionState.M_IDLE: "Idle",
            arips_arm_msgs.msg.MotionState.M_BREAK: "Break",
            arips_arm_msgs.msg.MotionState.M_HOLD: "Hold",
            arips_arm_msgs.msg.MotionState.M_RAW_MOTORS: "Raw Motors",
            arips_arm_msgs.msg.MotionState.M_DIRECT_JOINTS: "Direct Effort",
            arips_arm_msgs.msg.MotionState.M_DIRECT_CONTROLLER: "Direct Controller",
            arips_arm_msgs.msg.MotionState.M_TRAJECTORY: "Trajectory"
        }

        state = "Unknown"
        if mode in stateDict:
            state = stateDict[mode]

        return "Connected: {} ({})".format(state, mode)

    def onModeRawPressed(self):
        if self.isConnected():
            msg = arips_arm_msgs.msg.MotionCommand()
            msg.command = arips_arm_msgs.msg.MotionCommand.CMD_RAW_MOTORS
            self.pubMotionCommand.publish(msg)

    def onSendSetpointsPressed(self):
        if self.isConnected():
            msg = arips_arm_msgs.msg.RawMotorCommand()
            for i in range(len(msg.raw_motor_power)):
                try:
                    msg.raw_motor_power[i] = float(self.tblJointStates.item(i, self.JINDEX_SETPOINT).text())
                except ValueError:
                    msg.raw_motor_power[i] = 0.0
                self.tblJointStates.item(i, self.JINDEX_SETPOINT).setText(str(msg.raw_motor_power[i]))

            self.pubRawMotor.publish(msg)

    def onModeControllerPressed(self):
        if self.isConnected():
            msg = arips_arm_msgs.msg.MotionCommand()
            msg.command = arips_arm_msgs.msg.MotionCommand.CMD_DIRECT_CONTROLLER
            self.pubMotionCommand.publish(msg)
