package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.control.instant
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import page.j5155.expressway.ftc.motion.PIDFController

enum class SlidePosition(val position: Int) {
    DOWN(100),
    SPECIMEN_HANG(1000),
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
    var lastOutput: Double = 0.0
        private set

    constructor(hardwareMap: HardwareMap) : this(
        Motor.reversed(Motor("slidesLeft", hardwareMap)),
        Motor("slidesRight", hardwareMap)
    )

    fun setTarget(target: Int) =
        instant("set-slide-target-$target") { pid.targetPosition = target }

    fun setTarget(target: SlidePosition) = setTarget(target.position)

    fun goTo(target: Int) = Lambda("slide-go-to-$target")
        .setInit { setTarget(target).schedule() }
        .setFinish { currentPosition in target-10..target+10 }

    fun goTo(target: SlidePosition) = goTo(target.position)
}