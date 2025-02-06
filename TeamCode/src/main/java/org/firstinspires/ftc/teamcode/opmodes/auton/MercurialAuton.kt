package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot

@Autonomous
@Mercurial.Attach
class MercurialAuton : OpMode() {
    val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    lateinit var command: Command

    override fun init() {
        command = robot.drive.commandBuilder(Pose2d(0.0, 0.0, 0.0))
            .strafeTo(Vector2d(10.0, 0.0))
            .build()
    }

    override fun start() {
        command.schedule()
    }

    override fun loop() {
        telemetry.addData("Pose", robot.drive.localizer.pose)
        telemetry.update()
    }
}