package org.firstinspires.ftc.teamcode.hardware.mechanisms

import org.firstinspires.ftc.teamcode.control.services.PIDFService
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import page.j5155.expressway.ftc.motion.PIDFController

class LinearSlides(vararg motors: Motor) : List<Motor> by motors.toList() {
    companion object {
        @JvmField var kP = 0.01
        @JvmField var kI = 0.0
        @JvmField var kD = 0.1

        @JvmStatic val PIDF = PIDFController(PIDFController.PIDCoefficients(kP, kI, kD))
    }

    enum class SlidePosition(val position: Int) {
        DOWN(100),
        SPECIMEN_HANG(752),
        MIDDLE(2000),
        UP(3000),
    }

    val motors = motors.toList()
    val service = PIDFService(PIDF, *motors)
}