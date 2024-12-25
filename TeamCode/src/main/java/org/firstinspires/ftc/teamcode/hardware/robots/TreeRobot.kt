package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.control.convertToFieldCentric
import org.firstinspires.ftc.teamcode.control.fieldCentric
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.mechanisms.ServoArm
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class TreeRobot(hwMap: HardwareMap) {
    val drive = MecanumChassis(hwMap)
    val slides = LinearSlides(hwMap)
    val claw = TwoPointServo("claw", hwMap, 0.66, 1.0)
    val arm = ServoArm(hwMap)
    val otos: SparkFunOTOS
        get() = drive.otos

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
            gamepad.dpadRight.onTrue -> arm.goUp()
            gamepad.dpadLeft.onTrue -> arm.goDown()
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
                .fieldCentric(drive.pose.heading),
            gamepad.rightStickX.state
            )
    }
}