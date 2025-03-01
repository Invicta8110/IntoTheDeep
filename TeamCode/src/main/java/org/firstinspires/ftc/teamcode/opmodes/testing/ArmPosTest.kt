package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import org.firstinspires.ftc.teamcode.hardware.robots.position

@Autonomous(name = "ArmInit", group = "test")
class ArmPosTest : OpMode() {
    val arm by OpModeLazyCell { Elphabot(hardwareMap).arm }
    val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }

    val slidePid = LinearSlidesManual.PIDF

    override fun init() {
    }

    override fun loop() {
        when {
            gp1.a.onTrue -> arm.position = Elphabot.armUp
            gp1.b.onTrue -> arm.position = Elphabot.armDown
            gp1.x.onTrue -> arm.position = Elphabot.armHome
            gp1.y.onTrue -> arm.position = Elphabot.armBucket
        }

        telemetry.addData("arm position", arm.position)
    }


}