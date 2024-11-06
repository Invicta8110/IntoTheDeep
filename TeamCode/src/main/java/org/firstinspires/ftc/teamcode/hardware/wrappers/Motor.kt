package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.control.PIDFController
import org.firstinspires.ftc.teamcode.control.PIDFController.PIDCoefficients

class Motor(val internal: DcMotorEx) {
    //this can no longer be changed to java
    //testing

    inner class RTPAction(private val target: Int, val power: Double) : Action {
        private var initialized = false
        override fun run(p: TelemetryPacket): Boolean {
            if (!initialized) {
                internal.targetPosition = target
                internal.mode = DcMotor.RunMode.RUN_TO_POSITION
                internal.power = power

                initialized = true
            }

            return internal.isBusy
        }
    }

    inner class PIDFAction(private val target: Int, private val coefficients: PIDCoefficients) :
        Action {
        private var initialized = false
        private val pidf = PIDFController(coefficients)

        override fun run(p: TelemetryPacket): Boolean {
            if (!initialized) {
                pidf.setTargetPosition(target)
                initialized = true
            }

            internal.power = pidf.updateSquid(position.toDouble())

            return position in (target - 100)..(target + 100)
        }
    }

    init {
        internal.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        reset()
    }

    constructor(name: String, hwMap: HardwareMap) : this(hwMap.get(DcMotorEx::class.java, name))

    operator fun invoke(): DcMotorEx {
        return internal
    }

    val current get() = this().getCurrent(CurrentUnit.AMPS)
    val position get() = this().currentPosition

    fun runToPosition(target: Int, power: Double) {
        internal.targetPosition = target
        internal.mode = DcMotor.RunMode.RUN_TO_POSITION
        internal.power = power

        while (internal.isBusy) {
            //weewoo
        }

        //internal.setPower(0);
        //reset();
    }

    fun rtpAction(target: Int, power: Double): Action {
        return RTPAction(target, power)
    }

    fun pidfAction(target: Int, coefficients: PIDCoefficients): Action {
        return PIDFAction(target, coefficients)
    }

    fun reverse() = when (internal.direction) {
        DcMotorSimple.Direction.FORWARD -> internal.direction = DcMotorSimple.Direction.REVERSE
        DcMotorSimple.Direction.REVERSE -> internal.direction = DcMotorSimple.Direction.FORWARD
        null -> internal.direction = DcMotorSimple.Direction.FORWARD
    }

    fun reset() {
        internal.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        internal.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    companion object {
        fun reversed(motor: Motor): Motor {
            motor.reverse()
            return motor
        }
    }
}