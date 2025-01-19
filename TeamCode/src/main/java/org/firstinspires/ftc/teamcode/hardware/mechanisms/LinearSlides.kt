package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import page.j5155.expressway.core.actions.InitLoopAction
import page.j5155.expressway.ftc.motion.PIDFController

@Config
class LinearSlides(@get:JvmName("DOWN_POS") val DOWN_POS: Int,
                   @get:JvmName("UP_POS") val UP_POS: Int,
                   vararg motors: Motor)
    : List<Motor> by motors.toList() {
    val motors: List<Motor> = motors.toList()
    val position: Int
        get() = motors[0].currentPosition

    constructor(vararg motors: Motor) : this(0, 2000, *motors)

    constructor(hardwareMap: HardwareMap) : this(
        Motor.reversed(Motor("slidesLeft", hardwareMap)),
        Motor("slidesRight", hardwareMap)
    )

    fun reverse() = motors.forEach { m -> m.reverse() }

    fun up() {
        motors.forEach { m -> m().power = 1.0 }
    }

    fun setPower(power: Double) {
        motors.forEach { it().power = power }
    }

    fun powerAction(power: Double): InstantAction {
        return InstantAction { setPower(power) }
    }

    fun stop() {
        motors.forEach { it().power = 0.0 }
    }

    @get:JvmName("goUp")
    val goUp get() = ParallelAction(motors.map { it.RTPAction(UP_POS, 1.0) })

    @get:JvmName("goDown")
    val goDown get() = ParallelAction(motors.map { it.RTPAction(DOWN_POS, 1.0) })

    enum class SlidePosition(val position: Int) {
        DOWN(100), SPECIMEN_HANG(752), MIDDLE(2000), UP(3000)
    }

    companion object {
        @JvmField var kP = 0.01
        @JvmField var kI = 0.0
        @JvmField var kD = 0.1
    }

    fun runPID(initialTarget: SlidePosition) : SlidePIDAction {
        return SlidePIDAction(initialTarget)
    }

    inner class SlidePIDAction(private val initialTarget: SlidePosition) : InitLoopAction() {
        val pid = PIDFController(PIDFController.PIDCoefficients(kP, kI, kD))
        var enabled = true

        var target by pid::targetPosition

        override fun init() {
            pid.targetPosition = initialTarget.position
        }

        override fun loop(p: TelemetryPacket): Boolean {
            val output = pid.update(measuredPosition=motors[0].currentPosition.toDouble())

            if (enabled) motors.forEach { it.power = output }

            return true
        }

        fun updateTarget(target: Int) : Action {
            return InstantAction { pid.targetPosition = target }
        }

        fun goTo(target: Int) : Action {
            return object : Action {
                init {
                    enabled = true
                    pid.targetPosition = target
                }

                override fun run(p: TelemetryPacket) : Boolean {
                    return motors[0].currentPosition !in target-50..target+50
                }
            }
        }
    }
}