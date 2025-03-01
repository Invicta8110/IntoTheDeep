package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.DisplacementProfile
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.PosePath
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.project
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis

class FollowPathAction(val drive: MecanumChassis, val traj: DisplacementTrajectory) : Action {
    val path : PosePath = traj.path
    val profile : DisplacementProfile = traj.profile
    var disp = 0.0

    override fun run(p: TelemetryPacket) : Boolean {
        val robotVel: PoseVelocity2d = drive.updatePoseEstimate()
        disp = traj.project(drive.localizer.pose.position, disp)

        if (disp >= path.length() || (traj[traj.length()].position.value() - drive.localizer.pose.position).norm() < 2) {
            drive.setDrivePowers(0.0, 0.0, 0.0)
            return false
        }

        val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
            traj[disp],
            drive.localizer.pose,
            robotVel
        )

        drive.setDrivePowersWithFF(command)

        return true
    }
}

fun MecanumChassis.followPathCommand(traj: DisplacementTrajectory): Command {
    val path = traj.path
    val profile = traj.profile
    var disp = 0.0

    return Lambda("Following Path $traj")
        .setExecute {
            val robotVel = updatePoseEstimate()
            disp = traj.project(localizer.pose.position, disp)

            setDrivePowersWithFF(MecanumStatic.controller.compute(
                traj[disp],
                localizer.pose,
                robotVel
            ))
        }.setFinish {
            disp >= path.length() || (traj[traj.length()].position.value() - localizer.pose.position).norm() < 2
        }.setEnd {
            setDrivePowers(0.0, 0.0, 0.0)
        }
}

fun MecanumChassis.followPathCommand(traj: Trajectory) = this.followPathCommand(DisplacementTrajectory(traj))