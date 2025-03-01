package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.control.SilkRoad
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidePosition
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot

@TeleOp(name = "Mushroom With Actions", group = "Mushrooms")
@SilkRoad.Attach
class MushroomWithActions : OpMode() {
    val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }
    val slidePid by OpModeLazyCell { robot.slides.runPID(SlidePosition.DOWN) }

    val timer = ElapsedTime()
    var loopCount = 0

    override fun init() {
        timer.reset()
        slidePid.enabled = false
    }

    override fun start() {
        timer.reset()
        slidePid.enabled = true
        SilkRoad.runAsync(slidePid)
    }

    override fun loop() {
        SilkRoad.runAsync(robot.drive.drivePowerAction(
            PoseVelocity2d(
                Vector2d(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble()
                ),
                -gamepad1.right_stick_x.toDouble()
            )
        ))

        if (gp1.rightBumper.onTrue) {
            SilkRoad.runAsync(robot.claw.runToB)
        } else if (gp1.leftBumper.onTrue) {
            SilkRoad.runAsync(robot.claw.runToA)
        }

        if (gp1.dpadLeft.onTrue) {
            SilkRoad.runAsync(robot.wrist.runToB)
        } else if (gp1.dpadRight.onTrue) {
            SilkRoad.runAsync(robot.wrist.runToA)
        }

        // SLIDE ACTIONS (complicated)
        if (gp1.dpadUp.onTrue) {
            SilkRoad.runAsync(InstantAction{ slidePid.target = 2400 })
        } else if (gp1.dpadDown.onTrue) {
            SilkRoad.runAsync(InstantAction{ slidePid.target = 100 })
        }

        mtel.addData("Loop Time", timer.milliseconds()/loopCount)
        mtel.addData("Slide PID Target", slidePid.pid.targetPosition)
        mtel.addData("Slide PID Enabled", slidePid.enabled)
        mtel.addData("Robot Position", robot.drive.localizer.pose.position)
        mtel.addData("Robot Heading", robot.drive.localizer.pose.heading.log())
        mtel.update()

        loopCount++
    }

    fun autoDeposit() = SequentialAction(
        InstantAction { slidePid.enabled = true },
        ParallelAction(
            slidePid.goTo(2400),
        ),
        robot.claw.runToB
    )
}