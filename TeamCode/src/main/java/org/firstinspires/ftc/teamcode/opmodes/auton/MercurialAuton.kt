package org.firstinspires.ftc.teamcode.opmodes.auton

import android.util.Log
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@Autonomous
@Mercurial.Attach
class MercurialAuton : OpMode() {
    val robot by OpModeLazyCell { MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }
    lateinit var command: Command

    override fun init() {
        command = robot.commandBuilder(robot.localizer.pose)
            .strafeTo(Vector2d(10.0, 0.0))
            .build()
    }

    override fun start() {
        Log.e("WAVEDASH", "Current command: $command")
        command.schedule()
    }

    override fun loop() {
        telemetry.addData("Pose", robot.localizer.pose)
        telemetry.update()
    }
}