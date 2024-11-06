package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx


open class TwoPointServo @JvmOverloads constructor(
    private val servo: ServoImplEx,
    private val pA: Double = 0.0,
    private val pB: Double = 1.0,
) {

    val position get() = servo.position

    @get:JvmName("runToA")
    val runToA = InstantAction(::goToA)

    @get:JvmName("runToB")
    val runToB = InstantAction(::goToB)

    @JvmOverloads
    constructor(
        name: String,
        hwMap: HardwareMap,
        pA: Double = 0.0,
        pB: Double = 1.0,
    ) : this(
        hwMap.get(ServoImplEx::class.java, name),
        pA, pB
    )

    init {
        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servo.direction = Servo.Direction.FORWARD
    }

    fun goToA() {
        servo.position = pA
    }

    fun goToB() {
        servo.position = pB
    }

    fun runAction(): InstantAction {
        return if (servo.position == pA) {
            runToB
        } else {
            runToA
        }
    }
}