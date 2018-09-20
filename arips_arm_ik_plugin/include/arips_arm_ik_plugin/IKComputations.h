//
// Created by jgdo on 31.03.18.
//

#ifndef ARIPS_ARM_IK_PLUGIN_IKCOMPUTATIONS_H
#define ARIPS_ARM_IK_PLUGIN_IKCOMPUTATIONS_H

#include <tf/tf.h>

#include <cmath>

namespace arips_arm_plugins {

namespace IKComputations {

std::pair<tf::Quaternion, double> correctOrientation(tf::Quaternion const& orig, double baseRot) {
    tf::Vector3 baseNorm = tf::Vector3(0, 1, 0).rotate(tf::Vector3(0,0,1), baseRot);

    tf::Matrix3x3 matOrig(orig);
    tf::Vector3 toolX = matOrig.getColumn(0);

    double angleToolToPlane = toolX.angle(baseNorm);

    tf::Quaternion corrected;

    // TODO proper threshold
    if(std::abs(angleToolToPlane) < 0.001) {
        corrected = tf::Quaternion(tf::Vector3(0,0,1), -M_PI_2) * orig;
    } else {
        tf::Vector3 correctionAxis = baseNorm.cross(toolX).normalized();
        corrected = tf::Quaternion(correctionAxis, -angleToolToPlane + M_PI_2) * orig;
    }

    double tcpAngle = 0;

    // pointing up or down
    if(std::abs(toolX.angle(tf::Vector3(0,0,1)) - M_PI_2) < M_PI_2-0.001) {
        tf::Matrix3x3 matCorrected(corrected);
        tf::Vector3 tcpBaseUp = matCorrected.getColumn(0).cross(baseNorm).normalized();
        tf::Vector3 tcpLocalSide = matCorrected.getColumn(1);

        tcpAngle = std::atan2(tcpLocalSide.dot(tcpBaseUp), tcpLocalSide.dot(baseNorm));
    }

    return std::pair<tf::Quaternion, double>(corrected, tcpAngle);
}

float normAngle(float a) {
    return std::remainder(a, 2* M_PI);
}

void computeIK(tf::Vector3 const& posOrig, tf::Quaternion rotOrig, std::vector<double> linkLength, std::vector<double> &solution) {

    using v3 = tf::Vector3;
    using quat = tf::Quaternion;

    double j1 = std::atan2(posOrig.y(), posOrig.x());

    // ROS_INFO_STREAM("rotOrig: " << rotOrig.x() << ", " << rotOrig.y() << " " << rotOrig.z() << ", " << rotOrig.w());

    auto corrected = correctOrientation(rotOrig, j1);
    quat rot = corrected.first;
    
    // ROS_INFO_STREAM("corrected: " << rot.x() << ", " << rot.y() << " " << rot.z() << ", " << rot.w());

    double j5 = corrected.second;

    double lenTool = linkLength.at(4) + linkLength.at(5);

    tf::Matrix3x3 matRot(rot);
    v3 toolOffset = matRot.getColumn(0) * lenTool;

    v3 pos = posOrig - toolOffset;


    // ROS_INFO_STREAM("pos = " << pos.x() << " " << pos.y() << " " << pos.z());

    v3 posJ2Diff = pos - v3(0, 0, linkLength.at(0) + linkLength.at(1)) - v3(cos(j1) * 0.02, sin(j1)*0.02, 0);
    // ROS_INFO_STREAM("posJ2Diff = " << posJ2Diff.x() << " " << posJ2Diff.y() << " " << posJ2Diff.z());


    double len2 = linkLength.at(2);
    double len3 = linkLength.at(3);
    double dist = posJ2Diff.length();

    // ROS_INFO_STREAM("len2 = " << len2 << ", len3 = " << len3 << ", dist = " << dist);


    double j3 = 0;
    double offsetj2 = 0;

    if(std::abs(dist) < len2 + len3) {
        j3 = normAngle(M_PI - std::acos((len2*len2 + len3*len3 - dist*dist) / (2*len2*len3)));
        offsetj2 = std::acos((len2*len2 + dist*dist - len3*len3) / (2*len2*dist));
    }

    double basej2 = std::acos(posJ2Diff.z() / dist);

    // ROS_INFO_STREAM("offsetj2 = " << offsetj2 << ", basej2 = " << basej2);

    double j2 = basej2 - offsetj2;

    double j4From = j2 + j3 + M_PI_2;

    v3 basePlaneX(-cos(j1), -sin(j1), 0);
    v3 basePlaneY(0,0,1);

    double toolOffsetPlaneX = toolOffset.dot(basePlaneX);
    double toolOffsetPlaneY = toolOffset.dot(basePlaneY);

    double j4Base = std::atan2(toolOffsetPlaneY, toolOffsetPlaneX);

    double j4 = normAngle(j4Base - j4From);



    solution = {j1, j2, j3, j4, j5};

    // ROS_INFO_STREAM("j1 = " << j1 << ", j2 = " << j2 << ", j3 = " << j3 << ", j4 = " << j4 << ", j5 = " << j5);
    //ROS_INFO_STREAM("");

}

}

}

#endif //ARIPS_ARM_IK_PLUGIN_IKCOMPUTATIONS_H
