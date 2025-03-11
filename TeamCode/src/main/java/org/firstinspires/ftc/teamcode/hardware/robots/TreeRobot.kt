package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.control.fieldCentric
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.roadrunner.OTOSLocalizer

class TreeRobot(hwMap: HardwareMap) {
    val drive = MecanumChassis(hwMap)
    val slides = LinearSlidesManual(hwMap)
    val claw = TwoPointServo("claw", hwMap, 0.66, 1.0)
    val arm = TwoPointServo("armLeft", hwMap, 0.0, 1.0)
    val otos: SparkFunOTOS
        get() = (drive.mdLocalizer as OTOSLocalizer).otos

    fun slideManualControl(gamepad: SDKGamepad) {
        when {
            gamepad.dpadUp.active -> slides.setPower(0.75)
            gamepad.dpadDown.active -> slides.setPower(-0.75)
            else -> slides.setPower(0.0)
        }
    }

    fun clawManualControl(gamepad: SDKGamepad) {
        when {
            gamepad.rightBumper.onTrue -> claw.goToB()
            gamepad.leftBumper.onTrue -> claw.goToA()
        }
    }

    fun armManualControl(gamepad: SDKGamepad) {
        when {
            gamepad.dpadRight.onTrue -> arm.goToA()
            gamepad.dpadLeft.onTrue -> arm.goToB()
        }
    }

    fun driveManualControl(gamepad: SDKGamepad) {
        this.drive.setDrivePowers(
            gamepad.leftStickY.state,
            gamepad.leftStickX.state,
            gamepad.rightStickX.state
        )
    }

    fun driveManualControlFC(gamepad: SDKGamepad) {
        this.drive.setDrivePowers(
            Vector2d(gamepad.leftStickY.state, gamepad.leftStickX.state)
                .fieldCentric(drive.mdLocalizer.pose.heading),
            gamepad.rightStickX.state
            )
    }
}