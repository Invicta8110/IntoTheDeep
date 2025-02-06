package org.firstinspires.ftc.teamcode.hardware.robots;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

public class TestTeleOpRobot {
    private final Gamepad gp1;
    private final MecanumDrive drive;
    private final Motor arm;

    private final LinearSlides slides;
    private LynxModule controlHub, expansionHub;
    public TwoPointServo claw;

    public TestTeleOpRobot(HardwareMap hwMap, Gamepad gp1) {
        this.gp1 = gp1;
        drive = new MecanumDrive(hwMap, new Pose2d(0.0, 0.0, 0.0));
        slides = new LinearSlides(new Motor("slides", hwMap));
        arm = new Motor("armRight", hwMap);
        claw = new TwoPointServo("claw", hwMap);

        List<LynxModule> hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            if (module.isParent() && controlHub == null) {
                controlHub = module;
            } else if (expansionHub == null) {
                expansionHub = module;
            }
        }
    }

    public TestTeleOpRobot(HardwareMap hwMap, Gamepad gp1, MecanumDrive drive, LinearSlides slides, Motor arm) {
        this.gp1 = gp1;
        this.drive = drive;
        this.slides = slides;
        this.arm = arm;
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    public Motor getArm() {
        return arm;
    }

    public LinearSlides getSlides() {
        return slides;
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public LynxModule getExpansionHub() {
        return expansionHub;
    }

    public void slides() {
        if (gp1.a) {
            slides.setPower(1);
        } else if (gp1.b) {
            slides.setPower(-1);
        } else {
            slides.setPower(0);
        }
    }

    public void arm() {
        if (gp1.x) {
            arm.setPower(1);
        } else if (gp1.y) {
            arm.setPower(-1);
        } else {
            arm.setPower(0);
        }
    }

    public void test() {
        for (Motor m : slides.getMotors()) {
            m.runToPosition(5, 0.5);
        }
    }

    public void drive() {
        double y = -gp1.left_stick_y; // Remember, Y stick is reversed!
        double x = gp1.left_stick_x;
        double rx = gp1.right_stick_x;

        drive.leftFront.setPower(y + x + rx);
        drive.leftBack.setPower(y - x + rx);
        drive.rightFront.setPower(y - x - rx);
        drive.rightBack.setPower(y + x - rx);
    }

    public void claw() {
        if (gp1.a) {
            claw.goToA();
        } else if (gp1.b) {
            claw.goToB();
        }
    }
}
