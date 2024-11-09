package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class TestTeleOpRobot {
    private HardwareMap hwMap;
    private Gamepad gp1;
    private MecanumChassis drive;
    private Motor slides,arm;
    private Motor fLeft, fRight, bLeft, bRight;

    public TestTeleOpRobot(HardwareMap hwMap, Gamepad gp1) {
        this.hwMap = hwMap;
        this.gp1 = gp1;
        drive = new MecanumChassis(hwMap, new Pose2d(0.0,0.0,0.0));
        slides = new Motor("slides",hwMap);
        arm = new Motor("arm",hwMap);
        fLeft = new Motor("fLeft",hwMap);
        fRight = new Motor("fRight",hwMap);
        bLeft = new Motor("bLeft",hwMap);
        bRight = new Motor("bRight",hwMap);

        bRight.reverse();
        fRight.reverse();
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

    public void drive() {
        double y = -gp1.left_stick_y;
        double x = gp1.left_stick_x;
        double rx = gp1.right_stick_x;

        fLeft.getDcMotorEx().setPower(y + x + rx);
        bLeft.getDcMotorEx().setPower(y - x + rx);
        fRight.getDcMotorEx().setPower(y - x - rx);
        bRight.getDcMotorEx().setPower(y + x - rx);
    }

    public void test() {
        slides.runToPosition(5,.5);
    }
}
