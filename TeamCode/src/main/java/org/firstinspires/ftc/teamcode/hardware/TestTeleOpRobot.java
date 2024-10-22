package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class TestTeleOpRobot {
    private HardwareMap hwMap;
    private Gamepad gp1;
    private MecanumDrive drive;
    private Motor slides,arm;

    public TestTeleOpRobot(HardwareMap hwMap, Gamepad gp1) {
        this.hwMap = hwMap;
        this.gp1 = gp1;
        drive = new MecanumDrive(hwMap, new Pose2d(0.0,0.0,0.0));
        slides = new Motor("slides",hwMap);
        arm = new Motor("arm",hwMap);
    }

    public void slides() {
        if(gp1.a) {
            slides.getDcMotorEx().setPower(1);
        }
        else if(gp1.b) {
            slides.getDcMotorEx().setPower(-1);
        }
        else {
            slides.getDcMotorEx().setPower(0);
        }
    }

    public void arm() {
        if(gp1.x) {
            arm.getDcMotorEx().setPower(1);
        }
        else if(gp1.y) {
            arm.getDcMotorEx().setPower(-1);
        }
        else {
            arm.getDcMotorEx().setPower(0);
        }
    }

    public void test() {
        slides.runToPosition(5,.5);
    }
}
