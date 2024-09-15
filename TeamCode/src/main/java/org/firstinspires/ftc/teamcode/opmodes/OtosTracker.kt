package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive

@TeleOp
class OtosTracker : OpMode() {
    lateinit var robot: SparkFunOTOSDrive

    override fun init() {
        robot = SparkFunOTOSDrive(hardwareMap, Pose2d(0.0,0.0,0.0))
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