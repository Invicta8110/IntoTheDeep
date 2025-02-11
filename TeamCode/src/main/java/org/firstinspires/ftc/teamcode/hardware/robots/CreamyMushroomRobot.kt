package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesRR
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.roadrunner.OTOSLocalizer

@Config
class CreamyMushroomRobot
@JvmOverloads constructor(
    hwMap: HardwareMap,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val drive = MecanumChassis(hwMap, startPose)
    val slides = LinearSlidesRR(hwMap)
    val claw = TwoPointServo("claw", hwMap, 0.10, 0.625)
    val wrist = TwoPointServo("wrist", hwMap, 0.25, 0.50)
    val otos: SparkFunOTOS
        get() = (drive.localizer as OTOSLocalizer).otos

    val arm: List<ServoImplEx>
    val lynxes: List<LynxModule>

    init {
        val armRight = hwMap[ServoImplEx::class.java, "armRight"]!!
        val armLeft = hwMap[ServoImplEx::class.java, "armLeft"]!!

        armRight.pwmRange = armRange
        armLeft.pwmRange = armRange

        arm = listOf(armLeft, armRight)

        lynxes = hwMap.getAll(LynxModule::class.java)

        arm[1].direction = Servo.Direction.REVERSE
        // "thingy thingy thingy move"
        //      -andrew


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

    fun scoreSpecimen(slidePid: LinearSlidesRR.SlidePIDAction) = SequentialAction(
        slidePid.goTo(LinearSlidesRR.SlidePosition.SPECIMEN_HANG),
        claw.runToB,
        slidePid.goTo(LinearSlidesRR.SlidePosition.DOWN)
    )

    companion object {
        val armRange = PwmRange(500.0, 2500.0)

        val armConstant = 0.1
        val armHome = 0.64 // x wall grab from front
        val armUp = 0.12 // a enter submersible from front
        val armDown = 0.641 // b (keep)
        val armBucket = 0.572  // y up-right from front
    }
}

var List<ServoImplEx>.position: Double
    get() = this[1].position
    set(value) { this.forEach { it.position = value } }