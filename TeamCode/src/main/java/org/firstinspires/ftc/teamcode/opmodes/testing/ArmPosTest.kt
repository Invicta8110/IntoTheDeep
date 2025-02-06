package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot
import org.firstinspires.ftc.teamcode.hardware.robots.position

@Disabled
@TeleOp(name = "ArmInit", group = "test")
class ArmPosTest : OpMode() {
    val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }

    val slidePid = LinearSlides.PIDF

    override fun init() {
        robot.arm.position = CreamyMushroomRobot.armConstant
    }

    override fun loop() {

    }
}