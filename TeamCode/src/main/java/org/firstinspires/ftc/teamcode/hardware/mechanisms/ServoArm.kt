package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class ServoArm private constructor(private val left: TwoPointServo, private val right: TwoPointServo) {
    constructor(hwMap: HardwareMap, up: Double, down: Double) :
            this(TwoPointServo(hwMap[ServoImplEx::class.java, "armLeft"], up, down),
                 TwoPointServo(hwMap[ServoImplEx::class.java, "armRight"], 1-up, 1-down)
            )

    constructor(hwMap: HardwareMap) : this(hwMap, 1.0, 0.0)

    val upAction get() = InstantAction(::goUp)
    val downAction get() = InstantAction(::goDown)

    fun goUp() {
        left.goToA()
        right.goToA()
    }

    fun goDown() {
        left.goToB()
        right.goToB()
    }
}