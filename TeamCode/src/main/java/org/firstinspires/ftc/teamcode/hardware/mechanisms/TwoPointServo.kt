package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class TwoPointServo @JvmOverloads constructor (
    private val servo: Servo,
    private val pA: Double = 0.0,
    private val pB: Double = 1.0,
) {

    @JvmOverloads
    constructor(name: String,
                hwMap: HardwareMap,
                pA: Double = 0.0,
                pB: Double = 1.0,
    ) : this(
        hwMap.get(Servo::class.java, name),
        pA, pB
    )

    fun runToA() {
        servo.position = pA
    }

    fun runToB() {
        servo.position = pB
    }
}