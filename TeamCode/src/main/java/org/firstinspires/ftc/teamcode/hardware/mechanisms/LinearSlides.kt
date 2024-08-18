package org.firstinspires.ftc.teamcode.hardware.mechanisms

import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class LinearSlides(vararg motors: Motor) {
    private val motors: List<Motor>

    /**
     * NOTE: MOTORS MUST BE SET TO THE CORRECT DIRECTION BEFORE CONSTRUCTING LINEAR SLIDES
     */
    init {
        this.motors = motors.toList()
    }

    fun reverse() {
        motors.forEach { m -> m.reverse() }
    }

    fun up() {
        motors.forEach { m -> m().power = 1.0}
    }

}