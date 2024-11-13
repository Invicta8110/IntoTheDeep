package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.CPI_435_104
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class MecanumChassis
@JvmOverloads constructor(hwMap: HardwareMap, pose: Pose2d = Pose2d(0.0, 0.0, 0.0)) :
    MecanumDrive(hwMap, pose) {

    init {
        rightFront.direction = DcMotorSimple.Direction.REVERSE;
        rightBack.direction = DcMotorSimple.Direction.REVERSE;
    }

    inner class BadLocalizer {
        var lastPose: Pose2d

        var lastFLeft: Int
        var lastFRight: Int
        var lastBLeft: Int
        var lastBRight: Int

        init {
            lastPose = pose

            lastFLeft = leftFront.currentPosition
            lastFRight = rightFront.currentPosition
            lastBLeft = leftBack.currentPosition
            lastBRight = rightBack.currentPosition
        }

        fun update() : Pose2d {
            val deltaFLeft = (leftFront.currentPosition - lastFLeft) / CPI_435_104
            val deltaFRight = (rightFront.currentPosition - lastFRight) / CPI_435_104
            val deltaBLeft = (leftBack.currentPosition - lastBLeft) / CPI_435_104
            val deltaBRight = (rightBack.currentPosition - lastBRight) / CPI_435_104

            lastFLeft = leftFront.currentPosition
            lastFRight = rightFront.currentPosition
            lastBLeft = leftBack.currentPosition
            lastBRight = rightBack.currentPosition

            val deltaX = (deltaFLeft + deltaFRight + deltaBLeft + deltaBRight) / 4
            val deltaY = (-deltaFLeft + deltaFRight + deltaBLeft - deltaBRight) / 4
            val deltaHeading = (-deltaFLeft + deltaFRight - deltaBLeft + deltaBRight) / 4

            lastPose = Pose2d(lastPose.position + Vector2d(deltaX.toDouble(), deltaY.toDouble()), lastPose.heading + deltaHeading.toDouble())
            return lastPose
        }
    }

    val localizer: BadLocalizer = BadLocalizer()

    fun setDrivePowers(x: Double, y: Double, heading: Double) {
        setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))

    }

    fun moveToPointAction(x: Double, y: Double): Action {
        return actionBuilder(pose)
            .splineTo(Vector2d(x, y), pose.heading)
            .build()
    }

    fun turnAction(angle: Double): Action {
        return actionBuilder(pose)
            .turnTo(angle)
            .build()
    }

    fun followTrajectoryAction(trajectory: List<Pose2d>): Action {
        var traj = actionBuilder(pose)
        for (pose in trajectory) {
            traj = traj.splineTo(pose.position, pose.heading)
        }
        return traj.build()
    }

    fun drivePowerAction(x: Double, y: Double, heading: Double): Action {
        return Action {
            setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))
            false
        }
    }

    fun calculateFieldCentricPower(input: Vector2d): Vector2d {
        return pose.heading.inverse() * input
    }
}