package org.firstinspires.ftc.teamcode.opmodes.old.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;

@Disabled
@TeleOp
public class OtosTest extends OpMode {
    MecanumChassis robot;
    SparkFunOTOS otos;

    @Override
    public void init() {
        robot = new MecanumChassis(hardwareMap, new Pose2d(0, 0, 0));
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
    }

    @Override
    public void loop() {
        robot.setDrivePowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        Pose2d pose = Util.convertPoseToRR(otos.getPosition());

        Util.mtel.addData("Robot Position via OTOS", pose);
        Util.mtel.addData("Robot Position via RR", robot.localizer.getPose());
        Util.mtel.update();
    }
}
