package org.firstinspires.ftc.teamcode.opmodes.old.zauton;

import static org.firstinspires.ftc.teamcode.hardware.mechanisms.MotorArm.coefs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.SilkRoad;
import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.MotorArm;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

@Config
@SilkRoad.Attach
public class ArmAngleTest extends OpMode {
    //static PIDFController.PIDCoefficients coefs = new PIDFController.PIDCoefficients(0.01, 0, 0);

    SDKGamepad gp1;
    MotorArm motorArm;
    Motor.PIDFActionEx action;

    @Override
    public void init() {
        gp1 = new SDKGamepad(gamepad1);
        motorArm = new MotorArm(new Motor("arm", hardwareMap));

        action = motorArm.motor.pidfActionEx(0, coefs);
        SilkRoad.runAsync(action);
    }

    @Override
    public void loop() {
        if (gp1.dpadUp().onTrue()) {
            action.setTarget(100);
        } else if (gp1.dpadDown().onTrue()) {
            action.setTarget(0);
        }

        Util.mtel.addData("Arm Position", motorArm.motor.getCurrentPosition());
        Util.mtel.addData("Has arrived", action.isRunning());
        Util.mtel.update();
    }
}
