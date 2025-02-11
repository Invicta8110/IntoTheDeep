package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.timeout
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesRR
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot

@TeleOp
class SlidePidTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = CreamyMushroomRobot(hardwareMap)
        val slidePid = robot.slides.runPID(LinearSlidesRR.SlidePosition.DOWN)

        waitForStart()

        slidePid.enabled = true

        runBlocking(ParallelAction(
            slidePid.timeout(30.0),
            SequentialAction(
                slidePid.goTo(LinearSlidesRR.SlidePosition.UP),
                SleepAction(5.0),
                slidePid.goTo(LinearSlidesRR.SlidePosition.SPECIMEN_HANG),
            )
        ))
    }
}