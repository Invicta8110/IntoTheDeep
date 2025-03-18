package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import kotlin.math.PI

@Autonomous
@Mercurial.Attach
class StrafePls : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap) }

    override fun init() {}

    override fun start() {
        robot.drive.pathBuilder(robot.pose)
            .setTangent(PI/2)
            .lineToY(32.0)
            .build()
            .schedule()
    }

    override fun loop() {
        mtel.addData("Snapshot", Mercurial.activeCommandSnapshot)
        mtel.addData("Position", robot.pose.position)
        mtel.addData("Heading", robot.pose.heading.toDegrees())
        mtel.update()
    }
}