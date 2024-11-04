package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D

fun convertPoseToRR(pose: Pose2D) : Pose2d {
    return Pose2d(pose.x, pose.y, pose.h)
}