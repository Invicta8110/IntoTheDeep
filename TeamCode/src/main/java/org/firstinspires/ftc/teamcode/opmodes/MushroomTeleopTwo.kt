package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.control.SilkRoad
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot
import org.firstinspires.ftc.teamcode.hardware.robots.TreeRobot
import page.j5155.expressway.ftc.motion.PIDFController

@TeleOp(name = "Mushroom Two Drivers", group = "Mushrooms")
@Config
@SilkRoad.Attach
class MushroomTeleopTwo : OpMode() {
    private val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    private val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }
    private val gp2 by OpModeLazyCell { SDKGamepad(gamepad2) }
    private var slideCoefs = PIDFController.PIDCoefficients(LinearSlides.kP, LinearSlides.kI, LinearSlides.kD)
    private var slidePos = LinearSlides.SlidePosition.DOWN

    private val slidePid = PIDFController(slideCoefs)
    private var fieldCentric = false

    var loopCount = 0
    var timer = ElapsedTime()

    override fun init() {
        slidePid.targetPosition = 0
        timer.reset()
    }

    override fun loop() {
//        slideCoefs = PIDFController.PIDCoefficients(LinearSlides.kP, LinearSlides.kI, LinearSlides.kD)
//
//        robot.arm.pA = armA
//        robot.arm.pB = armB
//        robot.wrist.pA = wristA
//        robot.wrist.pB = wristB

        robot.driveManualControl(gamepad2)

        robot.drive.updatePoseEstimate()

        robot.clawManualControl(gamepad1)
        robot.wristManualControl(gp1)

        if (gp1.a.onTrue) {
            robot.arm.goToA()
        } else if (gp1.x.onTrue) {
            robot.arm.goToB()
        } else if (gp1.b.onTrue) {
            robot.arm.position = CreamyMushroomRobot.armCenter
        }

        if (gp1.dpadUp.state) {
            robot.slides.setPower(1.0)
        } else if (gp1.dpadUp.onFalse) {
            slidePid.targetPosition = robot.slides[0].currentPosition
        } else if (gp1.dpadDown.state) {
            robot.slides.setPower(-1.0)
        } else if (gp1.dpadDown.onFalse) {
            slidePid.targetPosition = robot.slides[0].currentPosition
        } else {
            val avg = robot.slides.sumOf { it.currentPosition } / robot.slides.size
            val output = slidePid.update(measuredPosition=robot.slides[0].currentPosition.toDouble())

            robot.slides[0].power = output
            robot.slides[1].power = output
        }

        robot.lynxes.forEachIndexed { i, it ->
            mtel.addData("Lynx $i Voltage", it.getInputVoltage(VoltageUnit.VOLTS))
            mtel.addData("Lynx $i Current", it.getCurrent(CurrentUnit.AMPS))
        }

        mtel.addData("Slide Position", robot.slides.position)
        mtel.addData("Robot Position", robot.drive.pose.position)
        mtel.addData("Robot Heading", robot.drive.pose.heading.log())
        mtel.addData("Loop Time", timer.milliseconds()/loopCount)
        mtel.update()

        loopCount++;
    }

    companion object {
        @JvmStatic var armA = 0.0
        @JvmStatic var armB = 1.0

        @JvmStatic var wristA = 0.0
        @JvmStatic var wristB = 1.0
    }
}