package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class HardwareGen(private val hwMap: HardwareMap) {
    fun buildMotor(name: String) : Motor {
        return Motor(name, hwMap)
    }

    fun buildServo(name: String) : Servo {
        return hwMap.get(Servo::class.java, name)
    }

    fun buildCRServo(name: String) : CRServo {
        return hwMap.get(CRServo::class.java, name)
    }

    fun buildDcEx(name: String) : DcMotorEx {
        return hwMap.get(DcMotorEx::class.java, name)
    }
}