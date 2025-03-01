package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.control.SilkRoad
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidePosition
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import org.firstinspires.ftc.teamcode.hardware.robots.position
import page.j5155.expressway.ftc.motion.PIDFController

@TeleOp(name="Elphaba Two Drivers", group = "Elphabot")
@Config
@SilkRoad.Attach
class ElphabaTwoDriver : OpMode() {
    private val robot by OpModeLazyCell { Elphabot(hardwareMap) }

    private val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }
    private val gp2 by OpModeLazyCell { SDKGamepad(gamepad2) }

    private var slideCoefs = PIDFController.PIDCoefficients(LinearSlidesManual.kP, LinearSlidesManual.kI, LinearSlidesManual.kD)
    private var slidePos = SlidePosition.DOWN

    private val slidePid = PIDFController(slideCoefs)
    private var fieldCentric = false
    var timer = ElapsedTime()
    var loopCount = 0;

    override fun init() {
        //robot.rotator.controller.pwmDisable()

        timer.reset()
    }

    override fun loop() {
        robot.drive.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                     gp2.leftStickY.state,
                    -gp2.leftStickX.state
                ),
                -gp2.rightStickX.state
            )
        )

        robot.drive.updatePoseEstimate()

        when {
            gp1.dpadLeft.onTrue -> robot.wrist.goTo(1)
            gp1.dpadRight.onTrue -> robot.wrist.goTo(0)
            gp1.b.onTrue -> robot.wrist.goTo(2)
        }

        when {
            gp1.rightBumper.onTrue -> robot.claw.goToB()
            gp1.leftBumper.onTrue -> robot.claw.goToA()
        }

        when {
            gp1.a.onTrue -> robot.arm.position = Elphabot.armPositions["a"]!!
            //gp1.b.onTrue -> robot.arm.position = Elphabot.armPositions["b"]!!
            gp1.x.onTrue -> robot.arm.position = Elphabot.armPositions["x"]!!
            gp1.y.onTrue -> robot.arm.position = Elphabot.armPositions["y"]!!
        }

        robot.rotator.power = gp1.rightTrigger.state - gp1.leftTrigger.state

        if (gp1.back.onTrue) {
            slidePid.targetPosition = SlidePosition.SPECIMEN_HANG.position
        }

        if (gp1.dpadUp.state) {
            robot.slidesManual.setPower(1.0)
        } else if (gp1.dpadUp.onFalse) {
            slidePid.targetPosition = robot.slidesManual[0].currentPosition
        } else if (gp1.dpadDown.state) {
            robot.slidesManual.setPower(-1.0)
        } else if (gp1.dpadDown.onFalse) {
            slidePid.targetPosition = robot.slidesManual[0].currentPosition
        } else {
            val output = slidePid.update(measuredPosition=robot.slidesManual[0].currentPosition.toDouble())

            robot.slidesManual[0].power = output
            robot.slidesManual[1].power = output
        }

        mtel.addData("Slide PID Target", slidePid.targetPosition)
        mtel.addData("Slide 0 Pos", robot.slidesManual[0].currentPosition)
        mtel.addData("Slide 1 Pos", robot.slidesManual[1].currentPosition)
        mtel.addData("Robot Position", robot.drive.localizer.pose.position)
        mtel.addData("Robot Heading", robot.drive.localizer.pose.heading.log())
        mtel.addData("Loop Time", timer.milliseconds()/loopCount)
        mtel.update()

        loopCount++;
    }
}