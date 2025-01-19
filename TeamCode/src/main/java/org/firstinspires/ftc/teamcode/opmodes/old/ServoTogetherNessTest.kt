package org.firstinspires.ftc.teamcode.opmodes.old

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import org.firstinspires.ftc.teamcode.hardware.robots.TreeRobot

@TeleOp
class ServoTogetherNessTest : OpMode() {
    val robot by OpModeLazyCell { TreeRobot(hardwareMap) }
    val gp1 by OpModeLazyCell { SDKGamepad(gamepad1) }

    override fun init() {
        robot.arm.left.position = 0.5
        robot.arm.right.position = 0.5
    }

    override fun loop() {
        if (gp1.a.onTrue) {
            robot.arm.left.position = 0.0
            robot.arm.right.position = 1.0
        } else if (gp1.b.onTrue) {
            robot.arm.left.position = 1.0
            robot.arm.right.position = 0.0
        } else if (gp1.x.onTrue) {
            robot.arm.left.position = 0.5
            robot.arm.right.position = 0.5
        }
    }
}