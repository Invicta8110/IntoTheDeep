package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive

@TeleOp
class OtosTracker : OpMode() {
    val robot: SparkFunOTOSDrive by lazy {
        SparkFunOTOSDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
    }

    val arm: DcMotorEx by lazy {
        hardwareMap.get("arm") as DcMotorEx
    }

    override fun init() {
        robot
    }

    override fun loop() {
        robot.setDrivePowers(PoseVelocity2d(
            Vector2d((-gamepad1.left_stick_y).toDouble(), (-gamepad1.left_stick_x).toDouble()),
            gamepad1.right_stick_x.toDouble()
        ))

        telemetry.addData("Position:", robot.otos.position)
        telemetry.update()
    }
}