from pathlib import Path
from typing import List, Optional, Tuple
import arips_semantic_map_msgs.msg as smm
import genpy
import yaml
import math
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl, InteractiveMarkerFeedback
import rospy


def create_msg(msg_type, value) -> genpy.Message:
    package, name = msg_type.split("/")
    module = __import__(package + ".msg")
    class_ = getattr(module.msg, name)
    msg = class_()
    msg_from_yaml(value, msg)
    return msg


def msg_from_yaml(y, msg) -> None:
    slot_types = dict(zip(msg.__slots__, msg._slot_types))
    # assert set(msg.__slots__) == set(y.keys) # TODO is this needed?
    for key, value in y.items():
        field = getattr(msg, key)
        if isinstance(field, list):
            assert isinstance(value, list)
            field_type = slot_types[key]
            assert field_type.endswith("[]")
            # [:-2] to remove [] from field type
            setattr(msg, key, [create_msg(field_type[:-2], elem) for elem in value])
        elif isinstance(field, genpy.Message):
            msg_from_yaml(value, field)
        elif isinstance(field, genpy.TVal):
            field.secs = value["secs"]
            field.nsecs = value["nsecs"]
            pass
        else:
            setattr(msg, key, value)


class SemanticMap:
    def __init__(self):
        self._map = smm.SemanticMap()
        self.map_pub = rospy.Publisher("semantic_map", smm.SemanticMap, queue_size=1, latch=True)
        self.marker_server = InteractiveMarkerServer("semantic_map_markers")

    @property
    def map(self) -> smm.SemanticMap:
        return self._map

    @map.setter
    def map(self, new_map: smm.SemanticMap):
        if not isinstance(new_map, smm.SemanticMap):
            raise ValueError("Semantic map must be a SemanticMap")
        self._map = new_map
        self.publish_map()

    def save(self, filename: Path):
        with open(filename, "w") as file:
            # str(message) is yaml
            file.write(str(self.map))
            file.write("\n")

    def load(self, filename: Path):
        with open(filename, "r") as file:
            yaml_msg = yaml.safe_load(file)
            map = smm.SemanticMap()
            msg_from_yaml(yaml_msg, map)
            self.map = map

    def publish_map(self):
        self.map_pub.publish(self.map)

        all_markers: List[Tuple] = []

        for index, door in enumerate(self.map.doors):
            all_markers.append(self.create_door_point_control(index, door.pivot, 'pivot'))
            all_markers.append(self.create_door_point_control(index, door.extent, 'extent'))

        self.marker_server.clear()
        for marker, feedback in all_markers:
            self.marker_server.insert(marker, feedback)
        self.marker_server.applyChanges()
    
    def create_door_point_control(self, index, point, field_name):
        # create an interactive marker for our server
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name = f"door_{index}_{field_name}"

            int_marker.pose.position.x = point.x
            int_marker.pose.position.y = point.y
            int_marker.pose.position.z = 0.001 # to stand out in rviz
            int_marker.pose.orientation.w = 1
            int_marker.pose.orientation.y = 1

            # create a grey box marker
            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.scale.x = 0.1
            sphere.scale.y = 0.1
            sphere.scale.z = 0.1
            sphere.color.r = 0.0
            sphere.color.g = 0.5
            sphere.color.b = 0.5
            sphere.color.a = 1.0

            # create a control which will move the sphere
            move_control = InteractiveMarkerControl()
            move_control.name = "move_xy"
            move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            move_control.markers.append(sphere)

            # add the control to the interactive marker
            int_marker.controls.append(move_control)

            return int_marker, self.create_door_callback(index, point)
    
    def create_door_callback(self, door_index: int, point: Optional[smm.Point2D]):
        return lambda msg: self.interactive_marker_door_callback(msg, door_index, point)

    def interactive_marker_door_callback(self, feedback: InteractiveMarkerFeedback, door_index: int, point: Optional[smm.Point2D]):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        assert feedback.header.frame_id == "map"

        print(f"------------- setting pose for door {door_index} ----------------")
        print(feedback)
        point.x = feedback.pose.position.x
        point.y = feedback.pose.position.y

        self.publish_map()
