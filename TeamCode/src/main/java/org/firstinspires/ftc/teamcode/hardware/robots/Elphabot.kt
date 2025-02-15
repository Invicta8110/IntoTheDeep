package org.firstinspires.ftc.teamcode.hardware.robots

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot.Companion.armRange
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

typealias ServoArm = List<ServoImplEx>

class Elphabot(
    hwMap: HardwareMap,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val drive = MecanumChassis(hwMap, startPose)
    val slides = LinearSlides(Motor("slidesLeft", hwMap), Motor("slidesLeft", hwMap))
    val claw = TwoPointServo("claw", hwMap, 0.10, 0.625)
    val wrist = TwoPointServo("wrist", hwMap, 0.25, 0.50)
    val arm: ServoArm = listOf(
        hwMap.get(ServoImplEx::class.java, "armLeft").apply {
            pwmRange = armRange
        },
        hwMap.get(ServoImplEx::class.java, "armRight").apply {
            pwmRange = armRange
            direction = Servo.Direction.REVERSE
        }
    )

    companion object {
        val armPositions = mapOf(
            "x" to 0.64,
            "y" to 0.12,
            "a" to 0.641,
            "b" to 0.572
        )
    }

    fun moveArm(position: String) = instant("Move arm to $position") { arm.position = armPositions[position]!! }

    fun setDrivePowers(powers: PoseVelocity2d) = instant("Set drive powers to $powers") { drive.setDrivePowers(powers) }
    fun setDrivePowers(x: Double, y: Double, rx: Double) = setDrivePowers(PoseVelocity2d(Vector2d(x, y), rx))
}