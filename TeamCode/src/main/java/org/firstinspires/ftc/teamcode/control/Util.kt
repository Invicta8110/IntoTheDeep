@file:JvmName("Util")

package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D

@JvmField val CPR_312 = 537.7

@JvmField val CPR_435 = 384.5

fun convertPoseToRR(pose: Pose2D): Pose2d {
    return Pose2d(pose.x, pose.y, pose.h)
}