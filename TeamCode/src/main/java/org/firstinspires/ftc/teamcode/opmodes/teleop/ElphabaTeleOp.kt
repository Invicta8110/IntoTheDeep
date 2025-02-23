package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.control.mtel
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot
import org.firstinspires.ftc.teamcode.hardware.robots.position

@TeleOp(name="Elphabot TeleOp", group="Elphabot")
@Mercurial.Attach
class ElphabaTeleOp : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap) }
    val slidePid by OpModeLazyCell { LinearSlides.PIDF }
    val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }
    val elapsedTime = ElapsedTime()
    var loopCount = 0
    var output = 0.0

    override fun init() {
        slidePid.targetPosition = 0
    }

    override fun start() {
        elapsedTime.reset()
    }

    override fun loop() {
        robot.drive.setDrivePowers(
            gp1.leftStickY.state,
            -gp1.leftStickX.state,
            -gp1.rightStickX.state
        )

        when {
            gp1.rightBumper.onTrue -> robot.claw.goToB()
            gp1.leftBumper.onTrue -> robot.claw.goToA()
        }

        when {
            gp1.dpadLeft.onTrue -> robot.wrist.goToB()
            gp1.dpadRight.onTrue -> robot.wrist.goToA()
        }

        when {
            gp1.a.onTrue -> robot.arm.position = Elphabot.armPositions["a"]!!
            gp1.b.onTrue -> robot.arm.position = Elphabot.armPositions["b"]!!
            gp1.x.onTrue -> robot.arm.position = Elphabot.armPositions["x"]!!
            gp1.y.onTrue -> robot.arm.position = Elphabot.armPositions["y"]!!
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

        mtel.addData("command snapshot", Mercurial.activeCommandSnapshot)
        mtel.addData("elapsed time", elapsedTime.seconds())
        mtel.addData("average loop time", "${elapsedTime.seconds() / loopCount} seconds")
        mtel.addData("robot pose", robot.pose)
        mtel.addData("slide target", slidePid.targetPosition)
        mtel.addData("slide last output", output)

        loopCount++
    }
}