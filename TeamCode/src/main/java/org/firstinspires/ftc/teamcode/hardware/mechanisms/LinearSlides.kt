package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class LinearSlides(@get:JvmName("DOWN_POS") val DOWN_POS: Int,
                   @get:JvmName("UP_POS") val UP_POS: Int,
                   vararg motors: Motor) {
    val motors: List<Motor>
    val position: Int
        get() = motors[0].position

    /**
     * NOTE: MOTORS MUST BE SET TO THE CORRECT DIRECTION BEFORE CONSTRUCTING LINEAR SLIDES
     */
    init {
        this.motors = motors.toList()
    }

    constructor(vararg motors: Motor) : this(0, 2000, *motors)

    constructor(hardwareMap: HardwareMap) : this(
        Motor.reversed(Motor("slidesLeft", hardwareMap)),
        Motor("slidesRight", hardwareMap)
    )

    fun reverse() {
        motors.forEach { m -> m.reverse() }
    }

    fun up() {
        motors.forEach { m -> m().power = 1.0 }
    }

    operator fun get(index: Int): Motor {
        return motors[index]
    }

    fun setPower(power: Double) {
        motors.forEach { it().power = power }
    }

    fun powerAction(power: Double): InstantAction {
        return InstantAction { setPower(power) }
    }

    fun stop() {
        motors.forEach { it().power = 0.0 }
    }

    @get:JvmName("goUp")
    val goUp get() = ParallelAction(motors.map { it.RTPAction(UP_POS, 1.0) })

    @get:JvmName("goDown")
    val goDown get() = ParallelAction(motors.map { it.RTPAction(DOWN_POS, 1.0) })

    enum class SlidePosition {
        DOWN, UP
    }
}