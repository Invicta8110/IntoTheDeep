package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

public class TestingThings extends LinearOpMode {
    public void runOpMode() {
        MecanumChassis robot = new MecanumChassis(hardwareMap);

        waitForStart();

        Actions.runBlocking(robot.moveToPoint(12, 12));
    }
}
