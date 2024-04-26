cmake_minimum_required(VERSION 3.0.2)
project(arips_semantic_map_msgs)

find_package(catkin REQUIRED
    COMPONENTS geometry_msgs std_msgs message_generation
)

add_message_files(
   FILES
   Point2D.msg
   Door.msg
   SemanticMap.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime
)
