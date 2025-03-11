package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.mercurial.commands.groups.Advancing
import org.firstinspires.ftc.teamcode.control.instant


open class TwoPointServo @JvmOverloads constructor(
    private val servo: ServoImplEx,
    var pA: Double = 0.0,
    var pB: Double = 1.0,
) {

    var position by servo::position
    var pwmRange by servo::pwmRange

    @get:JvmName("runToA")
    val runToA
        get() = InstantAction(::goToA)

    @get:JvmName("runToB")
    val runToB
        get() = InstantAction(::goToB)

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
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servo.direction = Servo.Direction.FORWARD
    }

    fun goToA() {
        servo.position = pA
    }

    fun goToB() {
        servo.position = pB
    }

    val goToACommand get() = instant("go-to-A") { goToA() }
    val goToBCommand get() = instant("go-to-B") { goToB() }

    val advancing = Advancing(goToACommand, goToBCommand)

    fun runAction(): InstantAction {
        return if (servo.position == pA) {
            runToB
        } else {
            runToA
        }
    }
}