package org.firstinspires.ftc.teamcode.roadrunner.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OTOSLinearScalar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.addLine("OTOS Linear Scalar Tuner");
        telemetry.addLine("Press START, then push the robot on the a known distance.");
        telemetry.addLine("Then copy the scalar into SparkFunOTOSDrive.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("Uncorrected Distance Traveled Y", drive.pose.position.x);
            telemetry.addData("Uncorrected Distance Traveled X", drive.pose.position.y);
            telemetry.update();
        }


    }
}
