import os
import rospy
import rospkg

from python_qt_binding import loadUi

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import tiny_reconfigure.msg
import tiny_reconfigure.srv

class ReconfigureWidget(QWidget):
    sigGroupDefReceived = Signal(tiny_reconfigure.msg.GroupDef)
    sigParamDefReceived = Signal(tiny_reconfigure.msg.ParameterDef)
    
    def __init__(self, parent=None):        
        super(ReconfigureWidget, self).__init__(parent)
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('tiny_reconfigure'), 'resource', 'ReconfClientPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        # Give QObjects reasonable names
        self.setObjectName('ReconfClientPluginUi')
        self.btnReload.clicked.connect(self.onReloadPressed);
        
        self.currentTopicRoot = None
        self.pubGetParamDef = None
        self.subGroupDef = None
        self.subParamDef = None
        
        self.sigGroupDefReceived.connect(self.onGroupDefReceivedQt)
        self.sigParamDefReceived.connect(self.onParameterDefReceivedQt)
        
        self.onReloadPressed()

    def onReloadPressed(self):
        self.parameterTree.clear()
        if self.currentTopicRoot != self.editTopicRoot.text():
            self.currentTopicRoot = self.editTopicRoot.text()
    
            if not self.pubGetParamDef is None:
                self.pubGetParamDef.unregister()
    
            if not self.subGroupDef is None:
                self.subGroupDef.unregister()
    
            if not self.subParamDef is None:
                self.subParamDef.unregister()
            
            self.pubGetParamDef = rospy.Publisher(self.currentTopicRoot + "get_def", tiny_reconfigure.msg.GetDef, queue_size=10)
            self.subGroupDef = rospy.Subscriber("group_def", tiny_reconfigure.msg.GroupDef, self.onGroupDefReceived, queue_size=10)
            self.subParamDef = rospy.Subscriber("param_def", tiny_reconfigure.msg.ParameterDef, self.onParameterDefReceived, queue_size=10)

        rospy.sleep(0.5)
        #try:
        #   self.pubGetParamDef.wait_for_service(1)
        self.pubGetParamDef.publish(tiny_reconfigure.msg.GetDef()) # tiny_reconfigure.msg.GetDef.CMD_LIST_ALL, 0, 0
        #   print "srvGetParamDef returned"
        #except (rospy.ServiceException, rospy.ROSException) as ex:
        #    print("Service did not process request: " + str(ex))

    def onSendPressed(self, group, item):
        print "pressed group:%s item:%s value:%i" % (group.text(0), item.name, item.value)

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
        item = ParameterTreeItem(self, group, msg.param_name, "type")
        item.button.clicked.connect(lambda: self.onSendPressed(group, item))

    def getCreateGroupItem(self, index, num = None):
        if not num is None:
            while self.parameterTree.topLevelItemCount() > num:
                self.parameterTree.takeTopLevelItem(self.parameterTree.topLevelItemCount() - 1)
        
            while self.parameterTree.topLevelItemCount() < num:
                group = QTreeWidgetItem(self.parameterTree)
                self.parameterTree.addTopLevelItem(group)

        return self.parameterTree.topLevelItem(index)

    def getCreateParamItem(self, group, index, num=None):
        if not num is None:
            while group.childCount() > num:
                group.takeChild(group.childCount() - 1)

            if group.childCount() < num:
                group = ParameterTreeItem(self)
                self.parameterTree.insertTopLevelItem(index, group)
                return group

        return self.parameterTree.topLevelItem(index)


# ------------------------------------------------------------------------------
# Custom QTreeWidgetItem
# ------------------------------------------------------------------------------
class ParameterTreeItem(QTreeWidgetItem):
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

        ## Column 2 - SpinBox:
        self.spinBox = QSpinBox()
        self.spinBox.setValue(0)
        self.treeWidget().setItemWidget(self, 2, self.spinBox)

        ## Column 3 - Button:
        self.button = QPushButton()
        self.button.setText("Send")
        self.treeWidget().setItemWidget(self, 3, self.button)

        ## Signals
        # self.treeWidget().connect( self.button, SIGNAL("clicked()"), self.buttonPressed )

    @property
    def name(self):
        '''
        Return name ( 1st column text )
        '''
        return self.text(0)

    @property
    def value(self):
        '''
        Return value ( 2nd column int)
        '''
        return self.spinBox.value()

    def buttonPressed(self):
        '''
        Triggered when Item's button pressed.
        an example of using the Item's own values.
        '''
        # print "This Item name:%s value:%i" %( self.name,
        #                                      self.value )
        # self.reconfWidget.onSendPressed(self.parent, self)
