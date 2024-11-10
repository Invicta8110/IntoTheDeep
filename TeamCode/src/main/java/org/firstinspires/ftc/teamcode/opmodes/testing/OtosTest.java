package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class OtosTest extends OpMode {
    MecanumDrive robot;
    SparkFunOTOS otos;

    @Override
    public void init() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
    }

    @Override
    public void loop() {


        Pose2d pose = Util.convertPoseToRR(otos.getPosition());
    }
}
