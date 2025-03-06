package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesMercurial
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidePosition
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import org.firstinspires.ftc.teamcode.hardware.robots.position
import page.j5155.expressway.ftc.motion.PIDFController

@TeleOp(name="Elphaba Commands", group = "Elphabot")
@Config
@Mercurial.Attach
class ElphabaCommands : OpMode() {
    private val robot by OpModeLazyCell { Elphabot(hardwareMap) }
    private val slides by OpModeLazyCell { robot.slidesMercurial }

    private val gp1 by OpModeLazyCell { BoundGamepad(SDKGamepad(gamepad1)) }

    private val rightTrigger by OpModeLazyCell {
        gp1.rightTrigger.conditionalBindState()
            .greaterThan(0.0)
            .bind()
    }

    private var slideCoefs = PIDFController.PIDCoefficients(LinearSlidesManual.kP, LinearSlidesManual.kI, LinearSlidesManual.kD)
    private var slidePos = SlidePosition.DOWN

    private val slidePid = PIDFController(slideCoefs)
    var timer = ElapsedTime()
    var loopCount = 0;

    override fun init() {
        timer.reset()
        slidePid.targetPosition = 0
    }

    override fun start() {
        gp1.dpadLeft.onTrue(robot.wrist.goToCommand(1))
        gp1.dpadRight.onTrue(robot.wrist.goToCommand(0))
        gp1.b.onTrue(robot.wrist.goToCommand(2))

        gp1.rightBumper.onTrue(robot.claw.goToBCommand())
        gp1.leftBumper.onTrue(robot.claw.goToACommand())

        gp1.a.onTrue(instant("arm-to-A") { robot.arm.position = Elphabot.armPositions["a"]!! })
        //gp1.b.onTrue(instant("arm-to-B") { robot.arm.position = Elphabot.armPositions["b"]!! })
        gp1.x.onTrue(instant("arm-to-X") { robot.arm.position = Elphabot.armPositions["x"]!! })
        gp1.y.onTrue(instant("arm-to-Y") { robot.arm.position = Elphabot.armPositions["y"]!! })

        slides.pidEnabled = true
    }

    override fun loop() {
        robot.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                    gp1.leftStickY.state,
                    -gp1.leftStickX.state
                ),
                -gp1.rightStickX.state
            )
        ).schedule()

        robot.drive.updatePoseEstimate()

        robot.rotator.power = gp1.rightTrigger.state - gp1.leftTrigger.state

        when {
            gp1.dpadUp.onTrue -> {
                slidePos = slidePos.next()
                slides.setTarget(slidePos).schedule()
            }
            gp1.dpadDown.onTrue -> {
                slidePos = slidePos.previous()
                slides.setTarget(slidePos).schedule()
            }
            gp1.rightTrigger.state > 0 -> {
                slidePos = SlidePosition.SPECIMEN_HANG
                robot.scoreSpecimen.schedule()
            }
            gp1.leftTrigger.state > 0 && Mercurial.isScheduled(robot.scoreSpecimen) -> {
                robot.scoreSpecimen.cancel()
            }
        }

        if (slides.pidEnabled) {
            slides.motors.forEach { it.power = slides.pid.update(slides.currentPosition.toDouble()) }
        }

        mtel.addData("Slide PID Target", slides.pid.targetPosition)
        mtel.addData("Robot Position", robot.drive.localizer.pose.position)
        mtel.addData("Robot Heading", robot.drive.localizer.pose.heading.log())
        mtel.addData("Loop Time", timer.milliseconds()/loopCount)
        mtel.update()

        loopCount++;
    }
}

