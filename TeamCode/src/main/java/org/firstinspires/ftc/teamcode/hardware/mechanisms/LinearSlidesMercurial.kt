package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import page.j5155.expressway.ftc.motion.PIDFController

enum class SlidePosition(val position: Int) {
    DOWN(0),
    SUBMERSIBLE(250),
    SPECIMEN_HANG(1025),
    MIDDLE(2000),
    UP(3000);

    fun next() : SlidePosition {
        return SlidePosition.entries[(ordinal + 1) % SlidePosition.entries.size]
    }

    fun previous() : SlidePosition {
        return SlidePosition.entries[(ordinal - 1 + SlidePosition.entries.size) % SlidePosition.entries.size]
    }
}

class LinearSlidesMercurial(val motors: Set<Motor>) {
    companion object {
        @JvmField var kP = 0.01
        @JvmField var kI = 0.0
        @JvmField var kD = 0.1

        @JvmStatic val PIDF = PIDFController(PIDFController.PIDCoefficients(kP, kI, kD))
    }

    constructor(vararg motors: Motor) : this(motors.toSet())
    constructor(slide: LinearSlidesManual) : this(slide.motors.toSet())

    val pid = PIDF
    val currentPosition by motors.first()::currentPosition
    var pidEnabled = false
    var lastOutput = 0.0

    fun setTarget(target: Int) =
        instant("set-slide-target-$target") { pid.targetPosition = target }

    fun setTarget(target: SlidePosition) = setTarget(target.position)

    fun goTo(target: Int) = Lambda("slide-go-to-$target")
        .setInit { setTarget(target).schedule() }
        .setFinish { currentPosition in target-10..target+10 }

    fun goTo(target: SlidePosition) = goTo(target.position)

    val operatePid = Lambda("slide-pid")
        .setFinish { false }
        .setExecute {
            lastOutput = pid.update(currentPosition.toDouble())

            if (pidEnabled) {
                motors.forEach { it.power = lastOutput }
            }
        }
}
