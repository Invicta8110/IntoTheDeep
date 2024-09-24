package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx


class TwoPointServo @JvmOverloads constructor (
    private val servo: ServoImplEx,
    private val pA: Double = 0.0,
    private val pB: Double = 1.0,
) {

    @JvmOverloads
    constructor(name: String,
                hwMap: HardwareMap,
                pA: Double = 0.0,
                pB: Double = 1.0,
    ) : this(
        hwMap.get(ServoImplEx::class.java, name),
        pA, pB
    )

    init {
        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0);
    }

    fun runToA() {
        servo.position = pA
    }

    fun runToB() {
        servo.position = pB
    }
}