package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

@Autonomous
public class ArmPidTest extends OpMode {
    Motor arm;
    PIDFController pidf;
    int target;
    final int UP_POS = 500, DOWN_POS = 0;
    SDKGamepad gamepad;

    @Override
    public void init() {
        arm = new Motor(hardwareMap.get(DcMotorEx.class, "arm"));
        pidf = new PIDFController(new PIDFController.PIDCoefficients(1, 0, 0));

        gamepad = (SDKGamepad) Pasteurized.gamepad1();
        target = DOWN_POS;
        pidf.setTargetPosition(target);
    }

    @Override
    public void loop() {
        if (gamepad.dpadUp().onTrue()) {
            pidf.setTargetPosition(UP_POS);
        } else if (gamepad.dpadDown().onTrue()) {
            pidf.setTargetPosition(DOWN_POS);
        }

        arm.getInternal().setPower(pidf.update(arm.getPosition()));
    }
}
