package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.core.FeatureRegistrar
import org.firstinspires.ftc.teamcode.control.mtel
import page.j5155.expressway.core.actions.InitLoopCondAction
import page.j5155.expressway.ftc.motion.PIDFController

class Motor(private val internal: DcMotorEx) : DcMotorEx by internal {
    //this can no longer be changed to java
    //testing

    inner class RTPAction(private var target: Int, private val power: Double) : Action {
        private var initialized = false

        override fun run(p: TelemetryPacket): Boolean {
            if (!initialized) {
                this@Motor.targetPosition = target
                mode = DcMotor.RunMode.RUN_TO_POSITION
                this@Motor.power = power

                initialized = true
            }

            FeatureRegistrar.activeOpMode.telemetry.addData("Motor Info", "Name: $internal; Target: $target; Error ${target-currentPosition}")
            FeatureRegistrar.activeOpMode.telemetry.update()

            return internal.isBusy
        }
    }

    inner class PIDFAction(target: Int, val pidf: PIDFController) : Action {
        constructor(target: Int, coefs: PIDFController.PIDCoefficients) : this(target, PIDFController(coefs))

        private var initialized = false

        var target = target
            set(value) {
                pidf.targetPosition = value
                field = value
            }

        override fun run(p: TelemetryPacket): Boolean {
            if (!initialized) {
                pidf.targetPosition = target
                initialized = true
            }

            power = pidf.update(currentPosition.toDouble())

            return currentPosition !in (target - 50)..(target + 50)
        }
    }

    inner class PIDFActionEx(target: Int, val pidf: PIDFController
    ) : InitLoopCondAction({ currentPosition in (target - 50)..(target + 50)} )  {

        constructor(target: Int, coefs: PIDFController.PIDCoefficients) : this(target, PIDFController(coefs))

        var target = target
            set(value) {
                pidf.targetPosition = value
                field = value
            }



        override fun init() {
            pidf.targetPosition = (target)
        }

        override fun loop(p: TelemetryPacket) {
            mtel.addData("Motor Info", "Name: $internal; Target: $target; Error ${target-currentPosition}")
            mtel.update()

            power = pidf.update(currentPosition.toDouble())
        }

        fun update(target: Int) { this.target = target }
        fun updateAction(target: Int) : Action = InstantAction { update(target) }
    }

    init {
        val controller = internal.controller
        internal.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        reset()
    }

    constructor(name: String, hwMap: HardwareMap) : this(hwMap[DcMotorEx::class.java, name])

    operator fun invoke(): DcMotorEx {
        return internal
    }

    fun runToPosition(target: Int, power: Double) {
        this.targetPosition = target
        this.mode = DcMotor.RunMode.RUN_TO_POSITION
        this.power = power

        while (this.isBusy) {
            //weewoo
        }

    }

    fun rtpAction(target: Int, power: Double): Action {
        return RTPAction(target, power)
    }

    fun pidfActionEx(target: Int, coefficients: PIDFController.PIDCoefficients): PIDFActionEx {
        return PIDFActionEx(target, coefficients)
    }

    fun reverse() = when (direction) {
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

        fun reverse(motor: DcMotorEx) = when (motor.direction) {
            DcMotorSimple.Direction.FORWARD -> motor.direction = DcMotorSimple.Direction.REVERSE
            DcMotorSimple.Direction.REVERSE -> motor.direction = DcMotorSimple.Direction.FORWARD
            null -> motor.direction = DcMotorSimple.Direction.FORWARD
        }

        fun reversed(motor: DcMotorEx) : DcMotorEx {
            reverse(motor)
            return motor
        }
    }
}


