package org.firstinspires.ftc.teamcode.hardware.robots

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class Wood(hwMap: HardwareMap) {
    val drive = MecanumChassis(hwMap)
    val slides = LinearSlides(Motor("slides", hwMap))
    val claw = TwoPointServo("claw", hwMap, 0.75, 1.0)
    val arm = TwoPointServo("arm", hwMap, 1.0, 0.25)
    val otos: SparkFunOTOS
        get() = drive.otos
}