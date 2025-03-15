package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import java.lang.Math.toDegrees

@Autonomous
@Mercurial.Attach
class AndrewAuton : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }
    lateinit var drive1: Command

    override fun init() {
        robot.claw.goToBCommand.setRunStates(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE)
            .schedule()

        drive1 = robot.drive.commandBuilder(Pose2d(0.0, 0.0, 0.0))
            .stopAndAdd(robot.scoreSpecimen)
            .strafeTo(Vector2d(-20.0, 0.0))
            .build()
    }

    override fun start() {
        robot.slidesMercurial.pidEnabled = true

        robot.slidesMercurial.operatePid.schedule()

        Sequential(
            drive1,
            robot.claw.goToACommand,
            Wait(0.5),
            Parallel(
                robot.pickUp,
                robot.drive.moveToPoint(Vector2d(0.0, 20.0))
            ),
            robot.drive.moveToPoint(Vector2d(25.0, 20.0)),
            robot.drive.moveToPoint(Vector2d(25.0, 22.0)),
            robot.drive.moveToPoint(Vector2d(3.0, 22.0)),

        ).schedule()
    }

    override fun loop() {
        mtel.addData("Snapshot", Mercurial.activeCommandSnapshot)
        mtel.addData("Position", robot.pose.position)
        mtel.addData("Heading", toDegrees(robot.pose.heading))
        mtel.update()
    }

    private fun toDegrees(heading: Rotation2d): Double {
        return toDegrees(heading.log())

    }
}
