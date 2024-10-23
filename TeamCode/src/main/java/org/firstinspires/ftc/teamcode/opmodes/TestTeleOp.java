package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.TestTeleOpRobot;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    private TestTeleOpRobot robot;

    public void runOpMode() {
        robot = new TestTeleOpRobot(hardwareMap,gamepad1);
        robot.test();
        waitForStart();
        while(opModeIsActive()) {
            robot.slides();
            robot.arm();
        }
    }

}
