package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.core.FeatureRegistrar
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.control.ActionsEx
import org.firstinspires.ftc.teamcode.control.PIDFController
import org.firstinspires.ftc.teamcode.control.PIDFController.PIDCoefficients
import org.firstinspires.ftc.teamcode.control.mtel

class Motor(private val internal: DcMotorEx) {
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

            FeatureRegistrar.activeOpMode.telemetry.addData("Motor Info", "Name: $internal; Target: $target; Error ${target-position}")
            FeatureRegistrar.activeOpMode.telemetry.update()

            return internal.isBusy
        }
    }

    inner class PIDFAction(private var target: Int, private val coefficients: PIDCoefficients) :
        Action {
        private var initialized = false
        val pidf = PIDFController(coefficients)

        override fun run(p: TelemetryPacket): Boolean {
            if (!initialized) {
                pidf.setTargetPosition(target)
                initialized = true
            }

            FeatureRegistrar.activeOpMode.telemetry.addData("Motor Info", "Name: $internal; Target: $target; Error ${target-position}")
            FeatureRegistrar.activeOpMode.telemetry.update()

            power = pidf.update(position.toDouble())

            return position in (target - 50)..(target + 50)
        }
    }

    inner class PIDFActionEx(private var target: Int, private val coefficients: PIDCoefficients,
    ) : ActionsEx({ position in (target - 50)..(target + 50)} )  {
        private val pidf = PIDFController(coefficients)

        override fun init() {
            pidf.setTargetPosition(target)
        }

        override fun loop() {
            mtel.addData("Motor Info", "Name: $internal; Target: $target; Error ${target-position}")
            mtel.update()

            power = pidf.update(position.toDouble())
        }

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

    val current: Double = internal.getCurrent(CurrentUnit.AMPS)
    val isBusy get() = internal.isBusy
    val position: Int by internal::currentPosition

    var power: Double by internal::power
    var targetPosition: Int by internal::targetPosition
    var direction: DcMotorSimple.Direction by internal::direction
    var mode: DcMotor.RunMode by internal::mode

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

    fun pidfAction(target: Int, coefficients: PIDCoefficients): Action {
        return PIDFAction(target, coefficients)
    }

    fun reverse() = when (direction) {
        DcMotorSimple.Direction.FORWARD -> internal.direction = DcMotorSimple.Direction.REVERSE
        DcMotorSimple.Direction.REVERSE -> internal.direction = DcMotorSimple.Direction.FORWARD
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


