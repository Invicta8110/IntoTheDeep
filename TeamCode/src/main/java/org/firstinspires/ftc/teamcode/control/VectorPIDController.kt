package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.max
import kotlin.math.min

class VectorPIDController @JvmOverloads constructor(
    val pid: PIDFController.PIDCoefficients,
    var targetPos: Vector2d,
    var targetVel: Vector2d = ZERO_VECTOR
) {

    var outputBounded = false
    var inputBounded = false

    var minInput = Double.POSITIVE_INFINITY
    var maxInput = Double.NEGATIVE_INFINITY
    lateinit var outputMin: Vector2d
    lateinit var outputMax: Vector2d

    var lastTimestamp: Long = 0
    var lastError = ZERO_VECTOR
    var errorSum = ZERO_VECTOR

    /**
     * Sets bound on the input of the controller. When computing the error, the min and max are
     * treated as the same value. (Imagine taking the segment of the real line between min and max
     * and attaching the endpoints.)
     *
     * @param min minimum input
     * @param max maximum input
     */
    @Deprecated("Not usable with this version of PIDFController")
    fun setInputBounds(min: Double, max: Double) {
        if (min < max) {
            inputBounded = true
            minInput = min
            maxInput = max
        }
    }

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    fun setOutputBounds(min: Vector2d, max: Vector2d) {
        if (min.compareTo(max) < 0) {
            outputBounded = true
            outputMin = min
            outputMax = max
        }
    }

    fun getPositionError(measuredPosition: Vector2d): Vector2d {
        val error = targetPos - measuredPosition
        return error
    }

    @JvmOverloads
    fun update(
        timestamp: Long = System.nanoTime(),
        measuredPosition: Vector2d,
        measuredVelocity: Vector2d? = null
    ): Vector2d {

        val errorP = getPositionError(measuredPosition)

        if (lastTimestamp.toInt() == 0) {
            lastTimestamp = timestamp
            lastError = errorP
            return ZERO_VECTOR
        }

        val dt = (timestamp - lastTimestamp).toDouble()
        errorSum += (errorP + lastError) * dt / 2.0
        val errorD = (errorP - lastError) / dt

        lastError = errorP
        lastTimestamp = timestamp

        val errorV = measuredVelocity?.let { targetVel - it } ?: errorD

        val output = (errorP * pid.kP) + (errorSum * pid.kI) + (errorD * pid.kD)

        return if (outputBounded) {
            vectorMax(outputMin, vectorMin(output, outputMax))
        } else {
            output
        }
    }
}



