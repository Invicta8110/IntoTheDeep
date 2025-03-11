package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class LineTest : LinearOpMode() {
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val action = drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
            .lineToX(32.0)
            .lineToX(0.0)
            .lineToX(16.0)
            .build()

        waitForStart()

        runBlocking(action)
    }
}