package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.HardwareFactory
import kotlin.math.abs

@TeleOp
class VegaOrientation : LinearOpMode() {
    override fun runOpMode() {
        val f = HardwareFactory(hardwareMap)
        val frontLeft = f.buildDcEx("frontLeft")
        val frontRight = f.buildDcEx("frontRight")
        val backLeft = f.buildDcEx("backLeft")
        val backRight = f.buildDcEx("backRight")
        val slides = f.buildDcEx("slides")

        slides.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()

        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x * 1.1 // Counteract imperfect strafing
            val rx = gamepad1.right_stick_x.toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = (abs(y) + abs(x) + abs(rx)).coerceAtLeast(1.0)
            val frontLeftPower = (y + x + rx) / denominator
            val backLeftPower = (y - x + rx) / denominator
            val frontRightPower = (y - x - rx) / denominator
            val backRightPower = (y + x - rx) / denominator

            frontLeft.power = frontLeftPower
            backLeft.power = backLeftPower
            frontRight.power = frontRightPower
            backRight.power = backRightPower

            val slidePower = gamepad1.right_trigger - gamepad1.left_trigger
            slides.power = slidePower.toDouble()


        }


    }
}