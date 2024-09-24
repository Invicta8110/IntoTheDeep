package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class TestRobotKt(hwMap: HardwareMap?) {
    var dt: MecanumDrive
    var slides: LinearSlides
    var intake: Motor
    var claw: TwoPointServo? = null
    var drone: TwoPointServo? = null

    init {
        val factory = HardwareGen(hwMap!!)
        dt = MecanumDrive(hwMap, Pose2d(0.0, 0.0, 0.0))
        val slidesLeft = factory.buildMotor("slidesLeft")
        slidesLeft.reverse()
        val slidesRight = factory.buildMotor("slidesRight")
        slides = LinearSlides(slidesLeft, slidesRight)
        intake = factory.buildMotor("intake")
    }

    fun teleop(gp: Gamepad) {
        dt.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                    gp.left_stick_y.toDouble(),
                    gp.left_stick_x.toDouble()
                ),
                gp.right_stick_y
                    .toDouble()
            )
        )
    }
}