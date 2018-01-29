#!/usr/bin/env python

#from tiny_reconfigure.srv import *
from tiny_reconfigure.msg import *
import rospy
import random

groupPub = None
paramPub = None

def handle_get_def(req):
    global groupPub, paramPub

    groupDefs = ["group 0", "group 1"]
    
    paramDefs = [[["param_0_0", ParameterDef.TYPE_INT, 42, 0]],
                 [["param_1_0", ParameterDef.TYPE_FLOAT, 0, 0.123],
                  ["param_1_1", ParameterDef.TYPE_INT, 13, 0],
                  ["param_1_2", ParameterDef.TYPE_FLOAT, 0, 2.34]]]
    
    if req.cmd == GetDef.CMD_LIST_ALL:
        print "handle_get_def CMD_LIST_ALL"
        
        groupPub.publish(GroupDef(1, 2, "group 1"))
        groupPub.publish(GroupDef(0, 2, "group 0"))
    
        if bool(random.getrandbits(1)):
            paramPub.publish(ParameterDef(group_id = 0,
                                          param_id = 0,
                                          num_params = 1,
                                          param_name = "param_0_0",
                                          type = ParameterDef.TYPE_INT,
                                          value_int = 42,
                                          value_float = 0))
    
        if bool(random.getrandbits(1)):
            paramPub.publish(ParameterDef(group_id=1,
                                          param_id=0,
                                          num_params=3,
                                          param_name="param_1_0",
                                          type=ParameterDef.TYPE_FLOAT,
                                          value_int=0,
                                          value_float=0.123))
    
        if bool(random.getrandbits(1)):
            paramPub.publish(ParameterDef(group_id=1,
                                          param_id=1,
                                          num_params=3,
                                          param_name="param_1_1",
                                          type=ParameterDef.TYPE_INT,
                                          value_int=13,
                                          value_float=0))
    
        if bool(random.getrandbits(1)):
            paramPub.publish(ParameterDef(group_id=1,
                                          param_id=2,
                                          num_params=3,
                                          param_name="param_1_2",
                                          type=ParameterDef.TYPE_FLOAT,
                                          value_int=0,
                                          value_float=2.34))
            
    elif req.cmd == GetDef.CMD_GET_GROUP_DEF:
        print "handle_get_def CMD_GET_GROUP_DEF " + str(req.group_id)
        groupPub.publish(GroupDef(group_id=req.group_id,
                                  num_groups=len(groupDefs),
                                  group_name=groupDefs[req.group_id]))
        
        for param_id in xrange(len(paramDefs[req.group_id])):
            paramPub.publish(ParameterDef(group_id=req.group_id,
                                          param_id=param_id,
                                          num_params=len(paramDefs[req.group_id]),
                                          param_name=paramDefs[req.group_id][param_id][0],
                                          type=paramDefs[req.group_id][param_id][1],
                                          value_int=23,
                                          value_float=2.34))
    elif req.cmd == GetDef.CMD_GET_PARAM_DEF:
        print "handle_get_def CMD_GET_PARAM_DEF " + str(req.group_id) + " " + str(req.param_id)
        paramPub.publish(ParameterDef(group_id=req.group_id,
                                      param_id=req.param_id,
                                      num_params=len(paramDefs[req.group_id]),
                                      param_name=paramDefs[req.group_id][req.param_id][0],
                                      type=paramDefs[req.group_id][req.param_id][1],
                                      value_int=paramDefs[req.group_id][req.param_id][2],
                                      value_float=paramDefs[req.group_id][req.param_id][3]))

# TODO def handleSetParam()

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Subscriber('get_def', GetDef, handle_get_def)
    global groupPub, paramPub
    groupPub = rospy.Publisher('group_def', GroupDef, queue_size=10)
    paramPub = rospy.Publisher('param_def', ParameterDef, queue_size=10)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
