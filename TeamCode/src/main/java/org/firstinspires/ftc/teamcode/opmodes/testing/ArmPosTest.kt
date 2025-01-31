package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot

@Disabled
@TeleOp(name = "ArmPosTest", group = "test")
class ArmPosTest : OpMode() {
    val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }

    val slidePid = LinearSlides.PIDF

    override fun init() {}

    override fun loop() {
        when {
            gp1.dpadUp.onTrue -> slidePid.targetPosition = LinearSlides.SlidePosition.UP.position
            gp1.dpadDown.onTrue -> slidePid.targetPosition = LinearSlides.SlidePosition.DOWN.position
        }
        
        when {
            gp1.a.onTrue -> robot.arm.position = 0.35
            gp1.b.onTrue -> robot.arm.position = 0.95 //home
            gp1.x.onTrue -> robot.arm.position = 0.60
        }

        val output = slidePid.update(measuredPosition=robot.slides.position.toDouble())
        robot.slides.setPower(output)

        telemetry.addData("Slide Target", slidePid.targetPosition)
        telemetry.addData("Slide Position", robot.slides.position)

        telemetry.addData("Arm Position", robot.arm.position)
    }
}