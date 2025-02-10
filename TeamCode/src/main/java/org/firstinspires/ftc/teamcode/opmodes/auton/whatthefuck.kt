package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import dev.frozenmilk.wavedash.DEFAULT_TRAJECTORY_PARAMS
import dev.frozenmilk.wavedash.Drive
import dev.frozenmilk.wavedash.Localizer
import dev.frozenmilk.wavedash.TrajectoryCommandBuilder
import org.firstinspires.ftc.teamcode.control.MecanumStatic.defaultAccelConstraint
import org.firstinspires.ftc.teamcode.control.MecanumStatic.defaultTurnConstraints
import org.firstinspires.ftc.teamcode.control.MecanumStatic.defaultVelConstraint
import org.firstinspires.ftc.teamcode.control.MecanumStatic.kinematics
import kotlin.math.PI

object FakeRobot : Drive {
    override val localizer= object : Localizer {
        override var pose: Pose2d = Pose2d(0.0, 0.0, 0.0)
        override fun update(): PoseVelocity2d {
            val twist = Twist2d(Vector2d(1.0, 1.0), PI/6)
            pose += twist
            return PoseVelocity2d(twist.line, twist.angle)
        }
    }

    override fun commandBuilder(beginPose: Pose2d): TrajectoryCommandBuilder {
        return TrajectoryCommandBuilder(
            { turn: TimeTurn -> this.turnCommand(turn) },
            { trajectory: TimeTrajectory -> this.followTrajectoryCommand(trajectory) },
            DEFAULT_TRAJECTORY_PARAMS,
            beginPose,
            0.0,
            defaultTurnConstraints,
            defaultVelConstraint,
            defaultAccelConstraint
        )
    }

    override fun followTrajectory(trajectory: TimeTrajectory, t: Double): Boolean {
        println("currently following $trajectory at $t")
        return t < trajectory.duration
    }

    override fun setDrivePowers(powers: PoseVelocity2d) {
        println("setting drive powers to go to $powers; this would be ${kinematics.inverse<Time>(PoseVelocity2dDual.constant(powers, 1))}")
    }

    override fun setDrivePowersWithFF(powers: PoseVelocity2d) {
        println(setDrivePowers(powers))
    }

    override fun turn(turn: TimeTurn, t: Double): Boolean {
        println("currently turning $turn at $t")
        return t < turn.duration
    }
}

class FakeOpMode {
    fun main() {
        val command = FakeRobot.commandBuilder(Pose2d(0.0, 0.0, 0.0))
            .strafeTo(Vector2d(1.0, 1.0))
            .build()

        while (!command.finished()) {
            command.execute()
        }
    }
}