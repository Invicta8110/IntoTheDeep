package org.firstinspires.ftc.teamcode.hardware.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.control.instant

class IndexServo(
    private val servo: ServoImplEx,
    vararg val positions: Double
) {
    var position by servo::position
    var pwmRange by servo::pwmRange

    constructor(
        name: String,
        hwMap: HardwareMap,
        vararg positions: Double
    ) : this(
        hwMap.get(ServoImplEx::class.java, name),
        *positions
    )

    init {
        require(positions.all { it in 0.0..1.0 })
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servo.direction = Servo.Direction.FORWARD
    }

    fun goTo(index: Int) { position = positions[index] }
    fun goToCommand(index: Int) = instant("go-to-$index") { goTo(index) }
}