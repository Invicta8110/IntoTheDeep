package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.control.SilkRoad
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.robots.TreeRobot
import page.j5155.expressway.ftc.motion.PIDFController

@TeleOp(name = "Single Driver TeleOp", group = "Tree Robot TeleOps")
@SilkRoad.Attach
class TreeTestTeleOp : OpMode() {
    private val robot by OpModeLazyCell { TreeRobot(hardwareMap) }
    private val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }
    private val gp2 by OpModeLazyCell { SDKGamepad(gamepad2) }

    private val slidePid = PIDFController(LinearSlides.PID)
    private var fieldCentric = false

    override fun init() {
        slidePid.targetPosition = 0
    }

    override fun loop() {
        if (gp1.leftStickButton.onTrue) {
            fieldCentric = !fieldCentric
        }

        when (fieldCentric) {
            true -> robot.driveManualControlFC(gp1)
            false -> robot.driveManualControl(gp1)
        }

        robot.drive.updatePoseEstimate()

        //robot.armManualControl(gp1)
        robot.clawManualControl(gp1)

        if (gp1.a.onTrue) {
            slidePid.targetPosition = LinearSlides.UP
        } else if (gp1.b.onTrue) {
            slidePid.targetPosition = LinearSlides.DOWN
        }

        robot.slides.forEachIndexed { i, it ->
            val output = slidePid.update(measuredPosition=it.currentPosition.toDouble())
            telemetry.addData("slides[$i] position", it.currentPosition)
            telemetry.addData("slides[$i] output", output)
            it.power = output
        }

        telemetry.addData("Slide PID Target", slidePid.targetPosition)
        telemetry.addData("Robot Position", robot.drive.pose)
        telemetry.update()
    }
}