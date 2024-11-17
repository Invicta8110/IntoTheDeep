@file:JvmName("Util")

package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import dev.frozenmilk.dairy.core.FeatureRegistrar
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round
import kotlin.math.sin

@JvmField val CPR_312 = 537.7
@JvmField val CPR_435 = 384.5
@JvmField val CPR_84 = 1993.6

@JvmField val CPI_312_104 = round((1 / (104 * PI)) * CPR_312 * 25.4).toInt()
@JvmField val CPI_435_104 = round((1 / (104 * PI)) * CPR_435 * 25.4).toInt()

@JvmField val ZERO_VECTOR = Vector2d(0.0, 0.0)

@JvmField val mtel: MultipleTelemetry = MultipleTelemetry(FeatureRegistrar.activeOpMode.telemetry, FtcDashboard.getInstance().telemetry)


fun Pose2D.convertPoseToRR(): Pose2d {
    return Pose2d(this.x, this.y, this.h)
}

fun Pose2d.convertPoseToOTOS(): Pose2D {
    return Pose2D(this.position.x, this.position.y, this.heading.toDouble())
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

fun convertToFieldCentric(pose: Pose2d): Pose2d {
    val heading = pose.heading.toDouble()
    val rotX = pose.position.x * cos(-heading) - pose.position.y * sin(-heading)
    val rotY = pose.position.x * sin(-heading) + pose.position.y * cos(-heading)
    return Pose2d(rotX, rotY, heading)
}