package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor


class LinearSlides(vararg motors: Motor) {
    private val motors: List<Motor>

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
        return object : Action {
            var initialized = false
            var arrived = false

            /**
             * Runs a single uninterruptible block. Returns true if the action should run again and false if it has completed.
             * A telemetry packet [p] is provided to record any information on the action's progress.
             */
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    motors.forEach { it().power = 1.0 }
                    initialized = true
                }

                val pos = motors[0]().currentPosition

                if (pos > 10000) {
                    motors.forEach { it().power = 0.0 }
                }

                return pos > 10000
            }
        }
    }

    fun downAction() : Action {
        return object : Action {
            var initialized = false
            var arrived = false

            /**
             * Runs a single uninterruptible block. Returns true if the action should run again and false if it has completed.
             * A telemetry packet [p] is provided to record any information on the action's progress.
             */
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    motors.forEach { it().power = -1.0 }
                    initialized = true
                }

                val pos = motors[0]().currentPosition

                if (pos < 100) {
                    motors.forEach { it().power = 0.0 }
                }

                return pos < 100
            }
        }
    }
}