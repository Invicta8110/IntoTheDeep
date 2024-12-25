package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import org.firstinspires.ftc.teamcode.control.SilkRoad
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@SilkRoad.Attach
@Autonomous
class TrajectorySequenceTester : OpMode() {
    val drive by OpModeLazyCell { MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }
    lateinit var seq: SequentialAction

    override fun init() {
        val trajs: List<Trajectory> = drive.sequenceBuilder(drive.pose)
            .splineTo(Vector2d(24.0, 24.0), Math.PI/2)
            .splineTo(Vector2d(24.0, 0.0), Math.PI/2)
            .build()

        val timeTrajs: List<TimeTrajectory> = List(trajs.size) {
            TimeTrajectory(trajs[it].path, TimeProfile(trajs[it].profile.baseProfile))
        }

        val actions: List<MecanumDrive.FollowTrajectoryAction> = timeTrajs.map { drive.FollowTrajectoryAction(it) }

        seq = SequentialAction(actions)
    }

    override fun start() {
        SilkRoad.runAsync(seq)
    }

    override fun loop() {
        //this doesn't need to do anything lmao
    }
}