package org.firstinspires.ftc.teamcode.control

interface PIDController<T> {
    val targetPosition: T
    val targetVelocity: T

    fun update(timestamp: Long, measuredPosition: T, measuredVelocity: T?): T

}