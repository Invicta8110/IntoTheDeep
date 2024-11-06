package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.ActionOpMode;
import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

@Config
@Autonomous
public class ArmPidTest extends ActionOpMode {
    public static int UP_POS = 500, DOWN_POS = 0;
    public static double kP = 0.01, kI = 0, kD = 0;
    Motor arm;
    PIDFController.PIDCoefficients pidf;

    @Override
    public void init() {
        arm = new Motor(hardwareMap.get(DcMotorEx.class, "arm"));
        pidf = new PIDFController.PIDCoefficients(kP, kI, kD);

        super.init();
    }

    @Override
    public void loop() {
        pidf.kD = kD;
        pidf.kI = kI;
        pidf.kP = kP;

        addActionOnPress(arm.pidfAction(UP_POS, pidf), gp1().dpadUp());
        addActionOnPress(arm.pidfAction(DOWN_POS, pidf), gp1().dpadDown());

        super.loop();
    }
}
