package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class SuperCoolThing : LinearOpMode() {
    override fun runOpMode() {
        val m = Motor("motor1", hardwareMap)

        waitForStart()

        m.dcMotorEx.power = 1.0
        m().power = 1.0
    }
}