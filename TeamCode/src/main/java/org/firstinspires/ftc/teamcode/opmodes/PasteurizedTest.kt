package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class PasteurizedTest : OpMode() {
    val drive: MecanumDrive by lazy { MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }
    val slides: LinearSlides by lazy { LinearSlides(hardwareMap) }
    val running = emptyList<Action>()

    //val gamepad = Pasteurized.gamepad1

    override fun init() {
        drive
        slides
    }

    override fun loop() {
        TODO("Not yet implemented")
    }
}