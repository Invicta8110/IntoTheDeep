package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.hardware.mechanisms.IndexServo
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesMercurial
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidePosition
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis

typealias ServoArm = List<ServoImplEx>

var ServoArm.position: Double
    get() = this[1].position
    set(value) {
        this.forEach { it.position = value }
    }

fun ServoArm.goToCommand(position: Double) =
    instant("arm-to-$position") { this.position = position }
fun ServoArm.goToCommand(position: String) =
    goToCommand(Elphabot.armPositions[position]!!)

class Elphabot(
    hwMap: HardwareMap,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val drive = MecanumChassis(hwMap, startPose)
    val claw = TwoPointServo("claw", hwMap, 0.10, 0.625)
    val wrist = IndexServo("wrist", hwMap, 0.35, 0.65, 0.05, 0.475)
    val rotator = IndexServo("rotator", hwMap, 0.20, 0.50, 0.80)

    val slidesManual = LinearSlidesManual(hwMap).apply {
        forEach { motor -> motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
        forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
    }
    val slidesMercurial = LinearSlidesMercurial(slidesManual)

    val arm = buildList {
        val armLeft = hwMap[ServoImplEx::class.java, "armLeft"]!!
        val armRight = hwMap[ServoImplEx::class.java, "armRight"]!!

        //armLeft.pwmRange = armRange
        //armRight.pwmRange = armRange

        armRight.direction = Servo.Direction.REVERSE

        add(armLeft)
        add(armRight)
    }

    private fun <E> MutableList<E>.addAll(vararg elements: E) {
        this.addAll(elements)
    }

    var pose
        get() = drive.mdLocalizer.pose
        set(value) {
            drive.mdLocalizer.pose = value
        }

    companion object {
        val armHome = 0.975 // x wall grab from front
        val armUp = 0.05 // a enter submersible from front
        val armBucket = 0.80  // y up-right from front
        val armSpecimen = 0.225 //

        val armPositions = mapOf(
            "x" to armHome,
            "a" to armUp,
            "b" to armSpecimen,
            "y" to armBucket,
            "spec" to armSpecimen
        )
    }

    fun moveArm(position: String) =
        instant("Move arm to $position") { arm.position = armPositions[position]!! }

    fun setDrivePowers(powers: PoseVelocity2d) =
        instant("Set drive powers to $powers") { drive.setDrivePowers(powers) }

    fun setDrivePowers(x: Double, y: Double, rx: Double) =
        setDrivePowers(PoseVelocity2d(Vector2d(x, y), rx))

    val scoreSpecimen get() = Parallel(
        arm.goToCommand("b"),
        wrist.goToCommand(3),
        rotator.goToCommand(2),
    )

    val pickUp get() = Parallel(
        arm.goToCommand("x"),
        wrist.goToCommand(0),
        rotator.goToCommand(0),
        claw.goToBCommand
    )
}