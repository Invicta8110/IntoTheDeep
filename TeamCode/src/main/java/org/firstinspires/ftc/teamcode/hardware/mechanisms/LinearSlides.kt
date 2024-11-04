package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor


class LinearSlides(vararg motors: Motor) {
    public val motors: List<Motor>

    /**
     * NOTE: MOTORS MUST BE SET TO THE CORRECT DIRECTION BEFORE CONSTRUCTING LINEAR SLIDES
     */
    init {
        this.motors = motors.toList()
    }

    constructor(hardwareMap: HardwareMap) : this(Motor.reversed(Motor("slidesLeft", hardwareMap)), Motor("slidesRight", hardwareMap))

    fun reverse() {
        motors.forEach { m -> m.reverse() }
    }

    fun up() {
        motors.forEach { m -> m().power = 1.0}
    }

    operator fun get(index: Int) : Motor {
        return motors[index];
    }

    fun setPower(power: Double) {
        motors.forEach { it().power = power }
    }

    fun powerAction(power: Double) : InstantAction {
        return InstantAction { setPower(power) }
    }

    fun upAction() : Action {
        return ParallelAction(motors.map {
            it.rtpAction(2000, 0.75)
        })
    }

    fun downAction() : Action {
        return ParallelAction(motors.map {
            it.rtpAction(0, 0.75)
        })
    }
}