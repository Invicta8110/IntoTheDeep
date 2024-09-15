package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class TestRobot {
    MecanumDrive dt;
    LinearSlides slides;
    Motor intake;
    TwoPointServo claw, drone;

    public TestRobot(HardwareMap hwMap) {
        HardwareGen factory = new HardwareGen(hwMap);

        dt = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));

        Motor slidesLeft = factory.buildMotor("slidesLeft");
        slidesLeft.reverse();
        Motor slidesRight = factory.buildMotor("slidesRight");
        slides = new LinearSlides(slidesLeft, slidesRight);

        intake = factory.buildMotor("intake");
    }

    public void teleop(@NonNull Gamepad gp) {
        dt.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(gp.left_stick_y,
                                gp.left_stick_x),
                        gp.right_stick_y
                )
        );
    }
}
