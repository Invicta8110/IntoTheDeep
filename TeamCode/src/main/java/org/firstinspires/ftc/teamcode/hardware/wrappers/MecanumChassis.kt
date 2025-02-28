package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.wavedash.DEFAULT_TRAJECTORY_PARAMS
import org.firstinspires.ftc.teamcode.control.MecanumStatic
import org.firstinspires.ftc.teamcode.control.MecanumStatic.defaultAccelConstraint
import org.firstinspires.ftc.teamcode.control.MecanumStatic.defaultVelConstraint
import org.firstinspires.ftc.teamcode.control.followPathCommand
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive


class MecanumChassis @JvmOverloads constructor(
    hwMap: HardwareMap,
    pose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) : MecanumDrive(hwMap, pose) {

    fun setDrivePowers(x: Double, y: Double, heading: Double) = setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))

    fun setDrivePowers(vector: Vector2d, heading: Double) = setDrivePowers(PoseVelocity2d(vector, heading))

    fun drivePowerAction(pose: PoseVelocity2d): Action = InstantAction { setDrivePowers(pose) }

    fun drivePowerAction(x: Double, y: Double, heading: Double): Action
        = InstantAction { setDrivePowers(x, y, heading) }

    fun trajectoryBuilder(beginPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(
            DEFAULT_TRAJECTORY_PARAMS,
            beginPose,
            0.0,
            defaultVelConstraint,
            defaultAccelConstraint
        )
    }

    fun moveToPoint(target: Pose2dDual<Time>) {
        val robotVel: PoseVelocity2d = this.updatePoseEstimate()
        val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
            targetPose = target,
            actualPose = this.localizer.pose,
            actualVelActual = robotVel
        )
        this.setDrivePowersWithFF(command)
    }

    fun moveToPoint(target: Pose2d): Command {
        return Lambda("Move to $target")
            .setExecute {
                val robotVel: PoseVelocity2d = this.updatePoseEstimate()
                val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
                    targetPose = Pose2dDual.constant(target, 3),
                    actualPose = this.localizer.pose,
                    actualVelActual = robotVel
                )

                this.setDrivePowers(command.value())
            }.setFinish {
                val error: Twist2d = target - this.localizer.pose
                error.line.norm() < 1.0 && error.angle < Math.PI/3
            }.setEnd {
                this.setDrivePowers(0.0, 0.0, 0.0)
            }
    }

    fun turnTo(target: Double): Command {
        return Lambda("Turn to $target rad")
            .setExecute {
                val robotVel: PoseVelocity2d = this.updatePoseEstimate()
                val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
                    targetPose = Pose2dDual.constant(Pose2d(0.0, 0.0, target), 3),
                    actualPose = this.localizer.pose,
                    actualVelActual = robotVel
                )
                this.setDrivePowers(command.value())
            }.setFinish {
                val error: Double = target - this.localizer.pose.heading.toDouble()
                error < Math.PI/8
            }
    }
}

fun List<Trajectory>.follow(robot: MecanumChassis): Command {
    return Sequential(this.map { robot.followPathCommand(it) })
}

//

