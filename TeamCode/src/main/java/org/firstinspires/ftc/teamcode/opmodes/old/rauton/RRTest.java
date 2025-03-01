package org.firstinspires.ftc.teamcode.opmodes.old.rauton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;


public class RRTest extends LinearOpMode {
    MecanumChassis robot;
    public void runOpMode() {
        waitForStart();
        TrajectoryActionBuilder action = robot.actionBuilder(robot.localizer.getPose()) //this is your starting position
                .splineTo(new Vector2d(0.0, 0.0), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(10.0, 10.0, 0), -Math.PI / 2)
                .splineTo(new Vector2d(0.0, -34.0), Math.PI / 2)
                .strafeTo(new Vector2d(-54.0, -34.0));
    }
}