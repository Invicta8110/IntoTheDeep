@file:JvmName("Util")

package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round

@JvmField val CPR_312 = 537.7
@JvmField val CPR_435 = 384.5
@JvmField val CPR_84 = 1993.6

@JvmField val CPI_312_104 = round((1 / (104 * PI)) * CPR_312 * 25.4).toInt()
@JvmField val CPI_435_104 = round((1 / (104 * PI)) * CPR_435 * 25.4).toInt()

@JvmField val ZERO_VECTOR = Vector2d(0.0, 0.0)

fun convertPoseToRR(pose: Pose2D): Pose2d {
    return Pose2d(pose.x, pose.y, pose.h)
}

fun Vector2d.compareTo(other: Vector2d): Int {
    val diff = this.norm() - other.norm()
    return when {
        diff > 0.0 -> 1
        diff < 0.0 -> -1
        else -> 0
    }
}

fun vectorMax(a: Vector2d, b: Vector2d): Vector2d {
    return Vector2d(max(a.x, b.x), max(a.y, b.y))
}

fun vectorMin(a: Vector2d, b: Vector2d): Vector2d {
    return Vector2d(min(a.x, b.x), min(a.y, b.y))
}