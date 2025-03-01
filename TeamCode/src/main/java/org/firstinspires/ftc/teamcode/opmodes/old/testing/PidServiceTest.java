package org.firstinspires.ftc.teamcode.opmodes.old.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.control.services.PIDFService;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.MotorArm;
import org.firstinspires.ftc.teamcode.hardware.robots.ChocolateRaisin;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import page.j5155.expressway.ftc.motion.PIDFController;

@Disabled
@Config
@TeleOp
public class PidServiceTest extends OpMode {
    public static PIDFService pidf;
    public static int UP_POS = 500, DOWN_POS = 0;

    MotorArm motorArm;
    SDKGamepad gp1;

    @Override
    public void init() {
        motorArm = new ChocolateRaisin(hardwareMap).getArm();
        gp1 = new SDKGamepad(gamepad1);

        pidf = new PIDFService(new PIDFController(new PIDFController.PIDCoefficients(0.01, 0, 0)), motorArm.motor);
        pidf.setTarget(0);
    }

    @Override
    public void loop() {
        pidf.setEnabled(gp1.leftBumper().onTrue());

        if (pidf.getEnabled()) {
            if (gp1.dpadUp().onTrue()) {
                pidf.setTarget(UP_POS);
            } else if (gp1.dpadDown().onTrue()) {
                pidf.setTarget(DOWN_POS);
            }
        } else {
            if (gp1.dpadUp().toggleTrue()) {
                motorArm.setPower(.5);
            } else if (gp1.dpadDown().toggleTrue()) {
                motorArm.setPower(-.5);
            } else {
                motorArm.setPower(0);
            }
        }

        Util.mtel.addData("Enabled", pidf.getEnabled());
        Util.mtel.addData("Target", pidf.getTarget());
        Util.mtel.addData("Position", motorArm.motor.getCurrentPosition());
        Util.mtel.addData("Power", motorArm.motor.getPower());
        Util.mtel.update();
    }
}
