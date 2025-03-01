package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.control.services.PIDFService
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import page.j5155.expressway.ftc.motion.PIDFController

enum class SlidePosition(val position: Int) {
    DOWN(100),
    SPECIMEN_HANG(752),
    MIDDLE(2000),
    UP(3000),
}

class LinearSlides(vararg motors: Motor) : List<Motor> by motors.toList() {
    companion object {
        @JvmField var kP = 0.01
        @JvmField var kI = 0.0
        @JvmField var kD = 0.1

        @JvmStatic val PIDF = PIDFController(PIDFController.PIDCoefficients(kP, kI, kD))
    }

    constructor(hardwareMap: HardwareMap) : this(
        Motor.reversed(Motor("slidesLeft", hardwareMap)),
        Motor("slidesRight", hardwareMap)
    )

    val motors = motors.toList()
    val service = PIDFService(PIDF, *motors)

    fun updateTarget(target: Int) : Command = instant("slide target $target") {
        service.target = target
        service.enabled = true
    }
    fun updateTarget(target: SlidePosition) = updateTarget(target.position)

    fun setPower(power: Double) = instant("slide power $power") {
        motors.forEach { it.power = power }
    }

    fun rawSetPower(power: Double) = motors.forEach { it.power = power }

    fun goTo(target: Int) : Command = Lambda("Go To $target")
        .setInit {
            service.enabled = true
            service.target = target
        }
        .setRequirements(this, service)
        .setFinish {
            service.atTarget(50)
        }
    fun goTo(target: SlidePosition) = goTo(target.position)
}