@file:JvmName("Util")

package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import kotlin.math.PI
import kotlin.math.round

@JvmField val CPR_312 = 537.7
@JvmField val CPR_435 = 384.5

@JvmField val CPI_312_104 = round((1 / (104 * PI)) * CPR_312 * 25.4).toInt()
@JvmField val CPI_435_104 = round((1 / (104 * PI)) * CPR_435 * 25.4).toInt()

fun convertPoseToRR(pose: Pose2D): Pose2d {
    return Pose2d(pose.x, pose.y, pose.h)
}