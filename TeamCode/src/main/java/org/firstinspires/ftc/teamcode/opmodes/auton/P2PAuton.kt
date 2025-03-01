package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot

@Autonomous
@Mercurial.Attach
class P2PAuton : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap) }
    val command by OpModeLazyCell {
        robot.drive.moveToPoint(Pose2d(12.0, 12.0, 0.0))
    }

    override fun init() {
        command.schedule()
    }

    override fun loop() {
        mtel.addData("Pose", robot.pose)
        mtel.update()
    }
}