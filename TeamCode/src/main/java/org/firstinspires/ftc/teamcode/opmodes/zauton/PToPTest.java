package org.firstinspires.ftc.teamcode.opmodes.zauton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.control.VectorPIDController;
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;

@Config
@Autonomous
public class PToPTest extends OpMode {
    public static PIDFController.PIDCoefficients coefs = new PIDFController.PIDCoefficients(0.01, 0, 0);
    public static Vector2d target = new Vector2d(24, 0);

    MecanumChassis dt;
    VectorPIDController pid;
    MultipleTelemetry mtel;

    @Override
    public void init() {
        dt = new MecanumChassis(hardwareMap);
        pid = new VectorPIDController(coefs, target);
        mtel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        Pose2d pose = dt.getLocalizer().update();
        Vector2d powers = pid.update(pose.position);

        dt.setDrivePowers(new PoseVelocity2d(powers, 0));

        mtel.addData("pose", pose);
        mtel.addData("target", target);
        mtel.addData("error", pid.getLastError());
        mtel.addData("powers", powers);
        mtel.update();
    }
}
