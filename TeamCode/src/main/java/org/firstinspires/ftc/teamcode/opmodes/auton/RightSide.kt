package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesRR
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot
import kotlin.math.PI

@Autonomous
class RightSide : LinearOpMode() {
    val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    val slidePid by OpModeLazyCell { robot.slides.runPID(LinearSlidesRR.SlidePosition.DOWN) }

    override fun runOpMode() {
        val action = robot.drive.actionBuilder(Pose2d(0.0, 0.0, PI/2))
            .strafeTo(Vector2d(8.0, 0.0))
            .build()

//        robot.drive.pose = redRight
//
//        val action = robot.drive.actionBuilder(redRight)
//            .splineToConstantHeading(redRight.position+18.0, PI/2)
//            .splineToConstantHeading(Vector2d(redRight.position.x+18, redRight.position.y), PI/2)
//            .build()

        waitForStart()

        runBlocking(action)
    }

//    override fun runOpMode() {
//        val action = robot.drive.actionBuilder(redRight)
//            .splineToSplineHeading(Pose2d(6.7, -36.1, PI/2), 0.0)
//            .afterDisp(0.0, slidePid.goTo(LinearSlides.SlidePosition.SPECIMEN_HANG))
//            .stopAndAdd(SequentialAction(
//                robot.claw.runToB,
//                slidePid.updateTarget(LinearSlides.SlidePosition.DOWN))
//            ).splineToSplineHeading(Pose2d(42.0, -60.0, PI/2), -PI/2)
//            .build()
//
//        waitForStart()
//
//        runBlocking(action)
//    }
}

private operator fun Vector2d.plus(d: Double) = Vector2d(x+d, y+d)
