package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Motor(val dcMotorEx: DcMotorEx) {
    //this should be changed to java
    //testing

    constructor(name: String, hwMap: HardwareMap) : this(hwMap.get(DcMotorEx::class.java, name))

    operator fun invoke() : DcMotorEx {
        return dcMotorEx
    }

    fun reset() {
        dcMotorEx.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        dcMotorEx.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun runToPosition(target: Int, power: Double) {
        dcMotorEx.targetPosition = target
        dcMotorEx.mode = DcMotor.RunMode.RUN_TO_POSITION
        dcMotorEx.power = power

        while (dcMotorEx.isBusy) {
            //it does stuff here
        }

        reset()
    }

    fun reverse() = when (dcMotorEx.direction) {
        DcMotorSimple.Direction.FORWARD -> dcMotorEx.direction = DcMotorSimple.Direction.REVERSE
        DcMotorSimple.Direction.REVERSE -> dcMotorEx.direction = DcMotorSimple.Direction.FORWARD
        null -> dcMotorEx.direction = DcMotorSimple.Direction.FORWARD
    }

}