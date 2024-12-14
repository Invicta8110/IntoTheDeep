package org.firstinspires.ftc.teamcode.opmodes.zauton;

import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm.coefs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.SilkRoad;
import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

@Config
@SilkRoad.Attach
public class ArmAngleTest extends OpMode {
    //static PIDFController.PIDCoefficients coefs = new PIDFController.PIDCoefficients(0.01, 0, 0);

    SDKGamepad gp1;
    Arm arm;
    Motor.PIDFActionEx action;

    @Override
    public void init() {
        gp1 = new SDKGamepad(gamepad1);
        arm = new Arm(new Motor("arm", hardwareMap));

        action = arm.motor.pidfActionEx(0, coefs);
        SilkRoad.runAsync(action);
    }

    @Override
    public void loop() {
        if (gp1.dpadUp().onTrue()) {
            action.setTarget(100);
        } else if (gp1.dpadDown().onTrue()) {
            action.setTarget(0);
        }

        Util.mtel.addData("Arm Position", arm.motor.getCurrentPosition());
        Util.mtel.addData("Has arrived", action.isRunning());
        Util.mtel.update();
    }
}
