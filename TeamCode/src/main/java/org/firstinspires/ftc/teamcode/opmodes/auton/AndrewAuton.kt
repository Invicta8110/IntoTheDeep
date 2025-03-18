package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.control.snapshotSize
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidePosition
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import java.lang.Math.toDegrees

@Autonomous
@Mercurial.Attach
class AndrewAuton : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }
    lateinit var drive1: Command
    lateinit var drive2: Command

    override fun init() {
        robot.claw.goToBCommand.setRunStates(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE)
            .schedule()

        val tab1 = robot.drive.pathBuilder(Pose2d(0.0, 0.0, 0.0))
            .afterTime(0.0, robot.scoreSpecimen)
            .strafeTo(Vector2d(-25.0, 0.0))
            .stopAndAdd(robot.claw.goToACommand)
            .stopAndAdd(robot.wrist.goToCommand(0))
            .strafeTo(Vector2d(-7.25, 0.0))
            .afterTime(2.0, Sequential(
                robot.wallGrab,
                robot.slidesMercurial.goTo(SlidePosition.DOWN)
            ))


        drive1 = tab1.build()
    }

    override fun start() {
        robot.slidesMercurial.pidEnabled = true
        Parallel(
            robot.slidesMercurial.operatePid,
            drive1
        ).schedule()
    }

    override fun loop() {
        mtel.addData("Snapshot", Mercurial.activeCommandSnapshot)
        mtel.addData("Snapshot Size", Mercurial.activeCommandSnapshot.snapshotSize())
        mtel.addData("Position", robot.pose.position)
        mtel.addData("Heading", robot.pose.heading.toDegrees())
        mtel.update()
    }
}

fun Rotation2d.toDegrees() = toDegrees(log())