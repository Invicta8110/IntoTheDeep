package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import org.firstinspires.ftc.teamcode.control.redRight
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.robots.CreamyMushroomRobot
import kotlin.math.PI

class RightSide : LinearOpMode() {
    val robot by OpModeLazyCell { CreamyMushroomRobot(hardwareMap) }
    val slidePid by OpModeLazyCell { robot.slides.runPID(LinearSlides.SlidePosition.DOWN) }

    override fun runOpMode() {
        val action = robot.drive.actionBuilder(redRight)
            .splineToSplineHeading(Pose2d(6.7, -36.1, PI/2), 0.0)
            .afterDisp(0.0, slidePid.goTo(LinearSlides.SlidePosition.SPECIMEN_HANG))
            .stopAndAdd(SequentialAction(
                robot.claw.runToB,
                slidePid.updateTarget(LinearSlides.SlidePosition.DOWN))
            ).splineToSplineHeading(Pose2d(42.0, -60.0, PI/2), -PI/2)
            .build()

        waitForStart()

        runBlocking(action)
    }
}