package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.Mercurial.gamepad1 as gp1
import org.firstinspires.ftc.teamcode.hardware.robots.Elphabot

@Mercurial.Attach
class ElphabaTeleOp : OpMode() {
    val robot by OpModeLazyCell { Elphabot(hardwareMap) }

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun start() {
        gp1.a.onTrue(robot.moveArm("a"))
        gp1.b.onTrue(robot.moveArm("b"))
        gp1.x.onTrue(robot.moveArm("x"))
        gp1.y.onTrue(robot.moveArm("y"))
    }

    override fun loop() {
        robot.setDrivePowers(
            -gp1.leftStickY.state,
            -gp1.leftStickX.state,
            -gp1.rightStickX.state
        ).schedule()
    }
}