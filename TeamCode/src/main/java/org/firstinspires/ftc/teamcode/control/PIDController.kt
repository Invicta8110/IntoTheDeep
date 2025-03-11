package org.firstinspires.ftc.teamcode.control

import kotlin.math.abs
import kotlin.math.withSign


/**
 * PID controller with various feedforward components.
 * Originally from Roadrunner 0.5
 * Ported to Kotlin by Zach.Waffle and j5155
 */
open class PIDController @JvmOverloads constructor(
    private var kP: Double,
     private var kD: Double = 0.0,
     private var kI: Double = 0.0
) {
    constructor(pid: PIDCoefficients, ) : this(pid.kP, pid.kD, pid.kI)

    /**
     * Target position (that is, the controller setpoint).
     */
    var targetPosition: Double = 0.0

    /**
     * Target velocity.
     */
    var targetVelocity: Double = 0.0

    /**
     * Target acceleration.
     */
    var targetAcceleration: Double = 0.0

    /**
     * Error computed in the last call to [update]
     */
    var lastError: Double = 0.0
    private var errorSum = 0.0
    private var lastUpdateTs: Long = 0
    private var inputBounded = false
    private var minInput = 0.0
    private var maxInput = 0.0
    private var outputBounded = false
    private var minOutput = 0.0
    private var maxOutput = 0.0



    /**
     * Sets bound on the input of the controller. When computing the error, the min and max are
     * treated as the same value. (Imagine taking the segment of the real line between min and max
     * and attaching the endpoints.)
     *
     * @param min minimum input
     * @param max maximum input
     */
    fun setInputBounds(min: Double, max: Double) {
        assert(min < max) { "Min output must be less than max output!" }
        inputBounded = true
        minInput = min
        maxInput = max
    }

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    fun setOutputBounds(min: Double, max: Double) {
        assert(min < max) { "Min output must be less than max output!" }
        outputBounded = true
        minOutput = min
        maxOutput = max
    }


    fun getPositionError(measuredPosition: Double): Double {
        var error = targetPosition - measuredPosition
        if (inputBounded) {
            val inputRange = maxInput - minInput
            while (abs(error) > inputRange / 2.0) {
                error -= inputRange.withSign(error)
            }
        }
        return error
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param timestamp        measurement timestamp as given by [System.nanoTime]
     * @param measuredPosition measured position (feedback)
     * @param measuredVelocity measured velocity
     */
    @JvmOverloads
    open fun update(
        timestamp: Long,
        measuredPosition: Double,
        measuredVelocity: Double? = null
    ): Double {
        val error = getPositionError(measuredPosition)

        if (lastUpdateTs == 0L) {
            lastError = error
            lastUpdateTs = timestamp
            return 0.0
        }

        val dt = (timestamp - lastUpdateTs).toDouble()
        errorSum += 0.5 * (error + lastError) * dt
        val errorDeriv = (error - lastError) / dt

        lastError = error
        lastUpdateTs = timestamp
        val velError = if (measuredVelocity == null) {
            errorDeriv
        } else {
            targetVelocity - measuredVelocity
        }

        return kP * error + kI * errorSum + kD * velError
    }

    open fun update(
        measuredPosition: Double
    ): Double {
        return update(System.nanoTime(),measuredPosition)
    }



    /**
     * Reset the controller's integral sum.
     */
    fun reset() {
        errorSum = 0.0
        lastError = 0.0
        lastUpdateTs = 0
    }

    // these must be var so that tuning with dash works
    data class PIDCoefficients(var kP: Double, var kI: Double, var kD: Double)
}
