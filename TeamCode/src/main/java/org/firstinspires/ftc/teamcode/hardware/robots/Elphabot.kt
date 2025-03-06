package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.hardware.mechanisms.IndexServo
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesMercurial
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidePosition
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot.Companion.armRange
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis

typealias ServoArm = List<ServoImplEx>

var ServoArm.position: Double
    get() = this[1].position
    set(value) {
        this.forEach { it.position = value }
    }

class Elphabot(
    hwMap: HardwareMap,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val drive = MecanumChassis(hwMap, startPose)
    val claw = TwoPointServo("claw", hwMap, 0.10, 0.625)
    val wrist = IndexServo("wrist", hwMap, 0.35, 0.65, 0.05)
    val rotator = hwMap[CRServo::class.java, "rotator"]

    val slidesManual = LinearSlidesManual(hwMap).apply {
        forEach { motor -> motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
        forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
    }
    val slidesMercurial = LinearSlidesMercurial(slidesManual)

    val arm = buildList {
        val armLeft = hwMap[ServoImplEx::class.java, "armLeft"]!!
        val armRight = hwMap[ServoImplEx::class.java, "armRight"]!!

        armLeft.pwmRange = armRange
        armRight.pwmRange = armRange

        armRight.direction = Servo.Direction.REVERSE

        add(armLeft)
        add(armRight)
    }

    private fun <E> MutableList<E>.addAll(vararg elements: E) {
        this.addAll(elements)
    }

    var pose
        get() = drive.localizer.pose
        set(value) {
            drive.localizer.pose = value
        }

    companion object {
        val armHome = 0.985 // x wall grab from front
        val armUp = 0.0 // a enter submersible from front
        val armDown = 0.80 // b (keep)
        val armBucket = 0.80  // y up-right from front

        val armPositions = mapOf(
            "x" to armHome,
            "a" to armUp,
            "b" to armDown,
            "y" to armBucket
        )
    }

    fun moveArm(position: String) =
        instant("Move arm to $position") { arm.position = armPositions[position]!! }

    fun setDrivePowers(powers: PoseVelocity2d) =
        instant("Set drive powers to $powers") { drive.setDrivePowers(powers) }

    fun setDrivePowers(x: Double, y: Double, rx: Double) =
        setDrivePowers(PoseVelocity2d(Vector2d(x, y), rx))

    val scoreSpecimen = Sequential(
        claw.goToBCommand(),
        instant("arm-to-Y") { arm.position = Elphabot.armPositions["y"]!! },
        slidesMercurial.goTo(SlidePosition.SPECIMEN_HANG),
        wrist.goToCommand(0),
        Wait(0.5),
        wrist.goToCommand(2),
        Wait(0.5),
        claw.goToACommand()
    )
}