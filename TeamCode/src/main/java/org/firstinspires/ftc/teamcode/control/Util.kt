@file:JvmName("Util")

package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import dev.frozenmilk.dairy.core.FeatureRegistrar
import org.firstinspires.ftc.teamcode.roadrunner.OTOSLocalizer.convertPose
import page.j5155.expressway.ftc.motion.PIDFController
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round
import kotlin.math.sin

const val CPR_312 = 537.7
const val CPR_435 = 384.5
const val CPR_84 = 1993.6

@JvmField val CPI_312_104 = round((1 / (104 * PI)) * CPR_312 * 25.4).toInt()
@JvmField val CPI_435_104 = round((1 / (104 * PI)) * CPR_435 * 25.4).toInt()

@JvmField val ZERO_VECTOR = Vector2d(0.0, 0.0)

@JvmField val mtel: MultipleTelemetry = MultipleTelemetry(FeatureRegistrar.activeOpMode.telemetry, FtcDashboard.getInstance().telemetry)

val blueRight = Pose2d(36.0, 60.0, -Math.PI/2)
val blueLeft = Pose2d(-26.0, 60.0, -Math.PI/2)
val redRight = Pose2d(26.0, -60.0, Math.PI/2)
val redLeft = Pose2d(-36.0, -60.0, Math.PI/2)

// Pose2d operations

fun Pose2D.convertPoseToRR(): Pose2d = convertPose(this)
fun Pose2d.convertPoseToOTOS(): Pose2D = convertPose(this)
fun Pose2d.convertToPoseVelocity2d() = PoseVelocity2d(this.position, this.heading.toDouble())

fun Pose2d.distanceTo(other: Pose2d) = this.position.distanceTo(other.position)

operator fun Pose2d.times(rotation: Rotation2d) = Pose2d(this.position * rotation, this.heading * rotation)

fun Pose2d.rotateBy(rotation: Rotation2d) = this * rotation
fun Pose2d.rotateBy(rotationRadians: Double) = this * Rotation2d.exp(rotationRadians)

// Vector2d operations

fun Vector2d.compareTo(other: Vector2d): Int {
    val diff = this.norm() - other.norm()
    return when {
        diff > 0.0 -> 1
        diff < 0.0 -> -1
        else -> 0
    }
}

fun Vector2d.distanceTo(other: Vector2d): Double = (this - other).norm()

operator fun Vector2d.times(rotation: Rotation2d) = rotation * this

fun Vector2d.rotateBy(rotation: Rotation2d) = this * rotation
fun Vector2d.rotateBy(rotationRadians: Double) = this * Rotation2d.exp(rotationRadians)

fun vectorMax(a: Vector2d, b: Vector2d): Vector2d {
    return Vector2d(max(a.x, b.x), max(a.y, b.y))
}

fun vectorMin(a: Vector2d, b: Vector2d): Vector2d {
    return Vector2d(min(a.x, b.x), min(a.y, b.y))
}

fun Vector2d.fieldCentric(rotation: Rotation2d): Vector2d = rotation.inverse() * this

fun convertToFieldCentric(vector: Vector2d, rotation: Rotation2d): Vector2d {
    return vector.fieldCentric(rotation)
}

fun PIDFController.updateInTolerance(
    tolerance: Double,
    timestamp: Long = System.nanoTime(),
    measuredPosition: Double,
    measuredVelocity: Double? = null,
    )
: Double {
    return when (getPositionError(measuredPosition)) {
        in -tolerance..tolerance -> {
            0.0
        }
        else -> {
            update(timestamp, measuredPosition, measuredVelocity)
        }
    }
}