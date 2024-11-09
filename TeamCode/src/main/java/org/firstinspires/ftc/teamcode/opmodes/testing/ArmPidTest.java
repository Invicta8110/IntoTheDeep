package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.ActionOpMode;
import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

@Config
@Autonomous
public class ArmPidTest extends ActionOpMode {
    public static double kP = 0.01, kI = 0, kD = 0;
    Arm arm;
    PIDFController pidf;
    PIDFController.PIDCoefficients coefs;
    MultipleTelemetry mtel;

    @Override
    public void init() {
        super.init();

        arm = new Arm(new Motor(hardwareMap.get(DcMotorEx.class, "arm")));
        arm.motor.reset();
        coefs = new PIDFController.PIDCoefficients(kP, kI, kD);
        pidf = new PIDFController(coefs);

        mtel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        coefs.kD = kD;
        coefs.kI = kI;
        coefs.kP = kP;

        addActionOnPress(arm.goUp(), gp1().dpadUp());
        addActionOnPress(arm.goDown(), gp1().dpadDown());

        mtel.addData("running:", getRunning());
        mtel.addData("count", arm.goUp().getCount());
        mtel.addData("position", arm.motor.getPosition());
        mtel.update();

        super.loop();
    }
}
