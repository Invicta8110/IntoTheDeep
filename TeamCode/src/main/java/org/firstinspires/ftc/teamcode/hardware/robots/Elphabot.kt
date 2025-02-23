package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot.Companion.armRange
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

typealias ServoArm = List<ServoImplEx>

var ServoArm.position: Double
    get() = this[1].position
    set(value) { this.forEach { it.position = value } }

class Elphabot(
    hwMap: HardwareMap,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val drive = MecanumChassis(hwMap, startPose)
    val slides = LinearSlides(hwMap)
    val claw = TwoPointServo("claw", hwMap, 0.10, 0.625)
    val wrist = TwoPointServo("wrist", hwMap, 0.25, 0.50)
    val arm by OpModeLazyCell {
        listOf(
            hwMap.get(ServoImplEx::class.java, "armLeft").apply {
                pwmRange = armRange
                direction = Servo.Direction.FORWARD
            },
            hwMap.get(ServoImplEx::class.java, "armRight").apply {
                pwmRange = armRange
                direction = Servo.Direction.REVERSE
            }
        )
    }

    init {
        slides.forEach { it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
        slides.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
    }

    private fun <E> MutableList<E>.addAll(vararg elements: E) {
        this.addAll(elements)
    }


    var pose
        get() = drive.localizer.pose
        set(value) { drive.localizer.pose = value }

    companion object {
        val armHome = 0.985 // x wall grab from front
        val armUp = 0.0 // a enter submersible from front
        val armDown = 0.65 // b (keep)
        val armBucket = 0.80  // y up-right from front

        val armPositions = mapOf(
            "x" to armHome,
            "a" to armUp,
            "b" to armDown,
            "y" to armBucket
        )
    }

    fun moveArm(position: String) = instant("Move arm to $position") { arm.position = armPositions[position]!! }

    fun setDrivePowers(powers: PoseVelocity2d) = instant("Set drive powers to $powers") { drive.setDrivePowers(powers) }
    fun setDrivePowers(x: Double, y: Double, rx: Double) = setDrivePowers(PoseVelocity2d(Vector2d(x, y), rx))
}