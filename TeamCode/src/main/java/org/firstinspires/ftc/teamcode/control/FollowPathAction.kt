package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.DisplacementProfile
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.PosePath
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.PositionPath
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.project
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.util.cell.AtomicCell
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis

class FollowPathAction(val drive: MecanumChassis, val traj: DisplacementTrajectory) : Action {
    val path : PosePath = traj.path
    val profile : DisplacementProfile = traj.profile
    var disp = 0.0

    override fun run(p: TelemetryPacket) : Boolean {
        val robotVel: PoseVelocity2d = drive.updatePoseEstimate()
        disp = traj.project(drive.mdLocalizer.pose.position, disp)

        if (disp >= path.length() || (traj[traj.length()].position.value() - drive.mdLocalizer.pose.position).norm() < 2) {
            drive.setDrivePowers(0.0, 0.0, 0.0)
            return false
        }

        val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
            traj[disp],
            drive.mdLocalizer.pose,
            robotVel
        )

        drive.setDrivePowersWithFF(command)

        return true
    }
}

fun MecanumChassis.followPathCommand(traj: DisplacementTrajectory): Command {
    var disp = 0.0

    return Lambda("Following-Path-${traj.repr}")
        .setInit {
            disp = 0.0
        }.setExecute {
            val robotVel = updatePoseEstimate()
            disp = traj.multiProject(mdLocalizer.pose.position, disp)

            setDrivePowersWithFF(controller.compute(
                traj[disp],
                mdLocalizer.pose,
                robotVel
            ))
        }.setFinish {
            disp >= traj.path.length() || (traj[traj.length()].position.value() - mdLocalizer.pose.position).norm() < 2
        }.setEnd {
            setDrivePowers(0.0, 0.0, 0.0)
        }
}

fun MecanumChassis.followPathCommand(traj: Trajectory) = this.followPathCommand(DisplacementTrajectory(traj))
fun MecanumChassis.followPathCommand(traj: TimeTrajectory) =
    this.followPathCommand(DisplacementTrajectory(traj.path, traj.profile.dispProfile))

fun PositionPath<Arclength>.project(query: Vector2d, init: Double = 0.0) : Double = project(this, query, init)

val DisplacementTrajectory.repr get() =
    "traj-${this.path.begin(1).value().position}-to-${this.path.end(1).value().position}"

fun multiProject(path: PosePath, query: Vector2d, init: Double, iter: Int = 5) =
    (1..iter).fold(init) { acc, _ ->
        project(path, query, acc)
    }

fun DisplacementTrajectory.multiProject(query: Vector2d, init: Double, iter: Int = 5) =
    multiProject(this.path, query, init, iter)