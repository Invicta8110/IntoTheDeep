package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot
import kotlin.math.PI

@Autonomous
class LeftSide : LinearOpMode() {
     val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }

    override fun runOpMode() {
        val action = robot.drive.actionBuilder(Pose2d(0.0, 0.0, PI/2))
            .strafeTo(Vector2d(16.0, 0.0))
            .build()

        waitForStart()

        runBlocking(action)
    }
}