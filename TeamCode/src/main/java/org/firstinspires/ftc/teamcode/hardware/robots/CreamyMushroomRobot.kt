package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis

@Config
class CreamyMushroomRobot

@JvmOverloads constructor(
    hwMap: HardwareMap,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val drive = MecanumChassis(hwMap, startPose)
    val slides = LinearSlides(hwMap)
    val claw = TwoPointServo("claw", hwMap, 0.15, 0.60)
    val arm = TwoPointServo("armRight", hwMap, armUp, armDown)
    val wrist = TwoPointServo("wrist", hwMap, 0.25, 0.55)
    val lynxes = hwMap.getAll(LynxModule::class.java)
    val otos: SparkFunOTOS
        get() = drive.otos

    init {
        arm.pwmRange = armRange
    }

    fun wristManualControl(gamepad: SDKGamepad) {
        when {
            gamepad.dpadLeft.onTrue -> wrist.goToB()
            gamepad.dpadRight.onTrue -> wrist.goToA()
        }
    }

    fun clawManualControl(gamepad: Gamepad) {
        when {
            gamepad.right_bumper -> claw.goToB()
            gamepad.left_bumper -> claw.goToA()
        }
    }

    fun driveManualControl(gamepad: Gamepad) {
        this.drive.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                    -gamepad.left_stick_y.toDouble(),
                    -gamepad.left_stick_x.toDouble()
                ),
                -gamepad.right_stick_x.toDouble()
            )
        )
    }

    companion object {
        val armRange = PwmRange(500.0, 2500.0)
        val armDown = 0.65
        val armUp = 0.20
        val armCenter = 0.40
    }
}