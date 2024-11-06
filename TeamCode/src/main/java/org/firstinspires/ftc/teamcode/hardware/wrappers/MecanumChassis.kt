package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class MecanumChassis
    @JvmOverloads constructor(hwMap: HardwareMap, pose: Pose2d = Pose2d(0.0, 0.0, 0.0))
    : MecanumDrive(hwMap, pose)
{

    fun moveToPointAction(x: Double, y: Double) : Action {
        return actionBuilder(pose)
            .splineTo(Vector2d(x, y), pose.heading)
            .build()
    }

    fun turnAction(angle: Double) : Action {
        return actionBuilder(pose)
            .turnTo(angle)
            .build()
    }

    fun followTrajectoryAction(trajectory: List<Pose2d>) : Action {
        var traj = actionBuilder(pose)
        for (pose in trajectory) {
            traj = traj.splineTo(pose.position, pose.heading)
        }
        return traj.build()
    }

    fun drivePowerAction(x: Double, y: Double, heading: Double) : Action {
        return Action {
            setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))
            false
        }
    }

    fun calculateFieldCentricPower(input: Vector2d) : Vector2d {
        return pose.heading.inverse() * input
    }
}