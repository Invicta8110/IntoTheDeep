package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot

@Autonomous
@Mercurial.Attach
class P2PAuton : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap, Pose2d(0.0, 0.0, Math.PI)) }

    override fun init() {}

    override fun start() {
        robot.drive.lineTo(Vector2d(10.0, 10.0)).schedule()
    }

    override fun loop() {
        mtel.addData("Robot Position", robot.drive.mdLocalizer.pose.position)
        mtel.addData("Robot Heading", robot.drive.mdLocalizer.pose.heading.log())
        mtel.update()
    }
}