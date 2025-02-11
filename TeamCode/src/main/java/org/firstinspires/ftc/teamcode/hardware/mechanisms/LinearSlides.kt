package org.firstinspires.ftc.teamcode.hardware.mechanisms

import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
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

    val motors = motors.toList()
    val service = PIDFService(PIDF, *motors)

    fun updateTarget(target: Int) : Command = Lambda("Slides To $target")
            .setInit {
                service.enabled = true
                service.target = target
            }
            .setRequirements(this, service)
            .setFinish { true }


    fun updateTarget(target: SlidePosition) = updateTarget(target.position)

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