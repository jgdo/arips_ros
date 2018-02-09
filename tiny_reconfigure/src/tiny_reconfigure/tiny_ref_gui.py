import os
import rospy
import rospkg

from python_qt_binding import loadUi

from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import tiny_reconfigure.msg
import tiny_reconfigure.srv

class ReconfigureWidget(QWidget):
    sigGroupDefReceived = Signal(tiny_reconfigure.msg.GroupDef)
    sigParamDefReceived = Signal(tiny_reconfigure.msg.ParameterDef)

    pkgpath = rospkg.RosPack().get_path('tiny_reconfigure')
    
    def __init__(self, parent=None):        
        super(ReconfigureWidget, self).__init__(parent)
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(self.pkgpath, 'resource', 'ReconfClientPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        # Give QObjects reasonable names
        self.setObjectName('ReconfClientPluginUi')
        self.btnReload.clicked.connect(self.onReloadPressed)
        self.btnSendAll.clicked.connect(self.onSendAllPressed)

        self.currentTopicRoot = None
        self.pubGetParamDef = None
        self.subGroupDef = None
        self.subParamDef = None
        self.srvSetParam = None
        
        self.sigGroupDefReceived.connect(self.onGroupDefReceivedQt)
        self.sigParamDefReceived.connect(self.onParameterDefReceivedQt)
        
        self.onReloadPressed()

    def onReloadPressed(self):
        if self.currentTopicRoot != self.editTopicRoot.text():
            self.currentTopicRoot = self.editTopicRoot.text()

            self.parameterTree.clear()
    
            if not self.pubGetParamDef is None:
                self.pubGetParamDef.unregister()
    
            if not self.subGroupDef is None:
                self.subGroupDef.unregister()
    
            if not self.subParamDef is None:
                self.subParamDef.unregister()
            
            self.pubGetParamDef = rospy.Publisher(self.currentTopicRoot + "get_def", tiny_reconfigure.msg.GetDef, queue_size=10)
            self.subGroupDef = rospy.Subscriber(self.currentTopicRoot + "group_def", tiny_reconfigure.msg.GroupDef, self.onGroupDefReceived, queue_size=10)
            self.subParamDef = rospy.Subscriber(self.currentTopicRoot + "param_def", tiny_reconfigure.msg.ParameterDef, self.onParameterDefReceived, queue_size=10)
            self.srvSetParam = rospy.ServiceProxy(self.currentTopicRoot + "set_param", tiny_reconfigure.srv.SetParam)

        rospy.sleep(0.5)
        #try:
        #   self.pubGetParamDef.wait_for_service(1)
        self.pubGetParamDef.publish(tiny_reconfigure.msg.GetDef(tiny_reconfigure.msg.GetDef.CMD_LIST_ALL, 0, 0))
        #   print "srvGetParamDef returned"
        #except (rospy.ServiceException, rospy.ROSException) as ex:
        #    print("Service did not process request: " + str(ex))

    def onGroupDefReceived(self, msg):
        print "onGroupDefReceived"
        self.sigGroupDefReceived.emit(msg)

    def onGroupDefReceivedQt(self, msg):
        print "onGroupDefReceivedQt"
        group = self.getCreateGroupItem(msg.group_id, msg.num_groups)
        group.setText(0, msg.group_name)

    def onParameterDefReceived(self, msg):
        self.sigParamDefReceived.emit(msg)
    
    def onParameterDefReceivedQt(self, msg):
        group = self.getCreateGroupItem(msg.group_id)
        if not group is None:
            item = self.getCreateParamItem(group, msg)
            item.value = msg

    def getCreateGroupItem(self, index, num = None):
        if not num is None:
            num = max(num, index+1)
            while self.parameterTree.topLevelItemCount() > num:
                self.parameterTree.takeTopLevelItem(self.parameterTree.topLevelItemCount() - 1)
        
            while self.parameterTree.topLevelItemCount() < num:
                group = GroupTreeItem(self.parameterTree, "<undefined>")
                self.parameterTree.addTopLevelItem(group)
                self.parameterTree.expandItem(group)
                group.btnRefresh.setIcon(QIcon(os.path.join(self.pkgpath, 'resource', 'reload.png')))
                group.btnRefresh.clicked.connect(lambda clicked, group=group: self.onRefreshGroupPressed(group))

        return self.parameterTree.topLevelItem(index)

    def getCreateParamItem(self, group, msg):
        if not msg.num_params is None:
            num = max(msg.num_params, msg.param_id+1)
            while group.childCount() > num:
                group.takeChild(group.childCount() - 1)

            while group.childCount() < num:
                item = ParameterTreeItem(group, "<undefined>", "<none>")
                item.btnSend.setIcon(QIcon(os.path.join(self.pkgpath, 'resource', 'send.png')))
                item.btnRefresh.setIcon(QIcon(os.path.join(self.pkgpath, 'resource', 'reload.png')))
                item.btnSend.clicked.connect(lambda clicked, item=item, group=group: self.onSendPressed(group, item))
                item.btnRefresh.clicked.connect(lambda clicked, item=item, group=group: self.onRefreshParamPressed(group, item))

        return group.child(msg.param_id)
    
    def onRefreshGroupPressed(self, group):
        self.pubGetParamDef.publish(tiny_reconfigure.msg.GetDef(tiny_reconfigure.msg.GetDef.CMD_GET_GROUP_DEF,
                                                                self.parameterTree.indexOfTopLevelItem(group),
                                                                0))
    
    def onSendPressed(self, group, item):
        value = item.value
        if (value is not None) and (item.valueChanged()):
            print "send group:%s item:%s value:%i" % (group.text(0), item.name, value)
            try:
                res = self.srvSetParam(self.parameterTree.indexOfTopLevelItem(group), group.indexOfChild(item), value, value)
                item.value = res
            except (rospy.ServiceException, rospy.ROSException) as ex:
                print("srvSetParam did not process request: " + str(ex))

    def onSendAllPressed(self):
        for group_id in xrange(self.parameterTree.topLevelItemCount()):
            group = self.parameterTree.topLevelItem(group_id)
            for param_id in xrange(group.childCount()):
                item = group.child(param_id)
                self.onSendPressed(group, item)

    
    def onRefreshParamPressed(self, group, item):
        self.pubGetParamDef.publish(tiny_reconfigure.msg.GetDef(tiny_reconfigure.msg.GetDef.CMD_GET_PARAM_DEF, self.parameterTree.indexOfTopLevelItem(group), group.indexOfChild(item)))

# ------------------------------------------------------------------------------
# Custom QTreeWidgetItem
# ------------------------------------------------------------------------------
class GroupTreeItem(QTreeWidgetItem):
    '''
    Custom QTreeWidgetItem with Widgets
    '''

    def __init__(self, parent, name):
        '''
        parent (QTreeWidget) : Item's QTreeWidget parent.
        name   (str)         : Item's name. just an example.
        '''

        ## Init super class ( QTreeWidgetItem )
        super(GroupTreeItem, self).__init__(parent)

        ## Column 0 - name:
        self.setText(0, name)
        
        ## Column 1 - Refresh Button:
        self.btnRefresh = QPushButton()
        self.btnRefresh.setText("Refresh")
        self.treeWidget().setItemWidget(self, 1, self.btnRefresh)



# ------------------------------------------------------------------------------
# Custom QTreeWidgetItem
# ------------------------------------------------------------------------------
class ParameterTreeItem(QTreeWidgetItem):
    paramTypeNames = {
        0: "<undefined>",
        tiny_reconfigure.msg.ParameterDef.TYPE_FLOAT: "float",
        tiny_reconfigure.msg.ParameterDef.TYPE_INT: "int",
    }
    
    '''
    Custom QTreeWidgetItem with Widgets
    '''

    def __init__(self, parent, name, typeStr):
        '''
        parent (QTreeWidget) : Item's QTreeWidget parent.
        name   (str)         : Item's name. just an example.
        '''

        ## Init super class ( QTreeWidgetItem )
        super(ParameterTreeItem, self).__init__(parent)

        ## Column 0 - name:
        self.setText(0, name)

        ## Column 1 - typeStr:
        self.setText(1, typeStr)

        ## Column 2 - value
        self.spinBox = None

        ## Column 3 - Send Button:
        self.btnSend = QPushButton()
        self.btnSend.setText("Send")
        self.btnSend.setEnabled(False)
        self.treeWidget().setItemWidget(self, 3, self.btnSend)

        ## Column 4 - Refresh Button:
        self.btnRefresh = QPushButton()
        self.btnRefresh.setText("Refresh")
        self.treeWidget().setItemWidget(self, 4, self.btnRefresh)

        self.remoteValue = None

        ## Signals
        # self.treeWidget().connect( self.button, SIGNAL("clicked()"), self.buttonPressed )

    @property
    def name(self):
        '''
        Return name ( 1st column text )
        '''
        return self.text(0)

    def resetChanged(self):
        if not self.spinBox is None:
            self.spinBox.setStyleSheet("")

    def setupSpinBox(self, spinBox):
        if isinstance(spinBox, QDoubleSpinBox):
            spinBox.setDecimals(5)
        self.spinBox = spinBox
        self.treeWidget().setItemWidget(self, 2, self.spinBox)
        self.spinBox.valueChanged.connect(self.onSpinboxValueChanged)

    def onSpinboxValueChanged(self):
        print "spinbox changed spin={} remote={}".format(self.spinBox.value(), self.remoteValue)
        if not self.spinBox is None:
            if self.valueChanged():
                self.spinBox.setStyleSheet("background-color: #CCFFCC")
            else:
                self.spinBox.setStyleSheet("")

    def valueChanged(self):
        if not self.spinBox is None:
            return abs(self.spinBox.value() - self.remoteValue) > 0.00001
        else:
            return False

    @property
    def value(self):
        if self.spinBox is None:
            return None
        return self.spinBox.value()

    @value.setter
    def value(self, msg):
        if isinstance(msg, tiny_reconfigure.msg.ParameterDef):
            self.setText(0, msg.param_name)
            self.setText(1, self.paramTypeNames[msg.type])

            if msg.type == tiny_reconfigure.msg.ParameterDef.TYPE_FLOAT:
                if not isinstance(self.spinBox, QDoubleSpinBox):
                    self.setupSpinBox(QDoubleSpinBox())

                self.remoteValue = msg.value_float
                self.spinBox.setValue(self.remoteValue)
                self.onSpinboxValueChanged()
                self.btnSend.setEnabled(True)
            elif msg.type == tiny_reconfigure.msg.ParameterDef.TYPE_INT:
                if not isinstance(self.spinBox, QSpinBox):
                    self.setupSpinBox(QSpinBox())

                self.remoteValue = msg.value_int
                self.spinBox.setValue(self.remoteValue)
                self.onSpinboxValueChanged()
                self.btnSend.setEnabled(True)
            else:
                self.spinBox = None
                self.treeWidget().setItemWidget(self, 2, self.spinBox)
                self.remoteValue = None
                self.btnSend.setEnabled(False)
        elif isinstance(msg, tiny_reconfigure.srv.SetParamResponse):
            if isinstance(self.spinBox, QDoubleSpinBox):
                self.remoteValue = msg.value_float
            elif isinstance(self.spinBox, QSpinBox):
                self.remoteValue = msg.value_int

            self.spinBox.setValue(self.remoteValue)
            self.onSpinboxValueChanged()
            self.btnSend.setEnabled(True)
