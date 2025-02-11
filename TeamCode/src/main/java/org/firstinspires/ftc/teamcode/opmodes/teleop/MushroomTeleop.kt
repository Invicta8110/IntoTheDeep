package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.control.SilkRoad
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesRR
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot
import org.firstinspires.ftc.teamcode.hardware.robots.position
import page.j5155.expressway.ftc.motion.PIDFController

@TeleOp(name = "Mushroom One Driver", group = "Mushrooms")
@Config
@SilkRoad.Attach
class MushroomTeleop : OpMode() {
    private val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    private val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }
    private var slideCoefs = PIDFController.PIDCoefficients(LinearSlidesRR.kP, LinearSlidesRR.kI, LinearSlidesRR.kD)
    private var slidePos = LinearSlidesRR.SlidePosition.DOWN

    private val slidePid = PIDFController(slideCoefs)
    private var fieldCentric = false
    var timer = ElapsedTime()
    var loopCount = 0;

    override fun init() {
        timer.reset()
        slidePid.targetPosition = 0
    }

    override fun loop() {
//        slideCoefs = PIDFController.PIDCoefficients(LinearSlides.kP, LinearSlides.kI, LinearSlides.kD)
//
//        robot.armRight.pA = armA
//        robot.armRight.pB = armB
//        robot.wrist.pA = wristA
//        robot.wrist.pB = wristB

        robot.driveManualControl(gamepad1)

        robot.drive.updatePoseEstimate()

        robot.clawManualControl(gamepad1)
        robot.wristManualControl(gp1)

        when {
            gp1.a.onTrue -> robot.arm.position = CreamyMushroomRobot.armUp
            gp1.x.onTrue -> robot.arm.position = CreamyMushroomRobot.armHome
            gp1.b.onTrue -> robot.arm.position = CreamyMushroomRobot.armDown
            gp1.y.onTrue -> robot.arm.position = CreamyMushroomRobot.armBucket
            gp1.leftTrigger.state > 0 -> robot.arm.position -= 0.0025
            gp1.rightTrigger.state > 0 -> robot.arm.position += 0.0025
        }

        if (gp1.back.onTrue) {
            slidePid.targetPosition = LinearSlidesRR.SlidePosition.SPECIMEN_HANG.position
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
            val output = slidePid.update(measuredPosition=robot.slides[0].currentPosition.toDouble())

            robot.slides[0].power = output
            robot.slides[1].power = output
        }

        mtel.addData("Slide PID Target", slidePid.targetPosition)
        mtel.addData("Slide 0 Pos", robot.slides[0].currentPosition)
        mtel.addData("Slide 1 Pos", robot.slides[1].currentPosition)
        mtel.addData("Robot Position", robot.drive.localizer.pose.position)
        mtel.addData("Robot Heading", robot.drive.localizer.pose.heading.log())
        mtel.addData("Loop Time", timer.milliseconds()/loopCount)
        mtel.update()

        loopCount++;
    }
}