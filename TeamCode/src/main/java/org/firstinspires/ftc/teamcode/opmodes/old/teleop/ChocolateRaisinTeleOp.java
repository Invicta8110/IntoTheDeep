package org.firstinspires.ftc.teamcode.opmodes.old.teleop;

import static org.firstinspires.ftc.teamcode.control.Util.mtel;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.robots.ChocolateRaisin;

import page.j5155.expressway.ftc.motion.PIDFController;

@Disabled
@TeleOp
public class ChocolateRaisinTeleOp extends OpMode {
    ChocolateRaisin robot;
    PIDFController slidesPid;
    Pose2d drivePowers;
    boolean fieldCentric = false;

    @Override
    public void init() {
        robot = new ChocolateRaisin(hardwareMap);
        slidesPid = new PIDFController(new PIDFController.PIDCoefficients(0.01, 0, 0));
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_button) {
            fieldCentric = !fieldCentric;
        }

        drivePowers = new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        if (fieldCentric) {
            drivePowers = new Pose2d(
                    Util.convertToFieldCentric(drivePowers.position, robot.getPose().heading),
                    drivePowers.heading
            );
        }
        robot.getChassis().setDrivePowers(new PoseVelocity2d(drivePowers.position, drivePowers.heading.toDouble()));

        if (gamepad1.right_bumper) {
            robot.getSlides().setPower(1);
        } else if (gamepad1.left_bumper) {
            robot.getSlides().setPower(-1);
        } else {
            robot.getSlides().setPower(0);
        }

        if (gamepad1.dpad_up) {
            robot.getArm().motor.setPower(.50);
        } else if (gamepad1.dpad_down) {
            robot.getArm().motor.setPower(-.50);
        } else {
            robot.getArm().motor.setPower(0);
        }


        if (gamepad1.right_trigger > 0) {
            robot.getClaw().goToB();
        } else if (gamepad1.left_trigger > 0) {
            robot.getClaw().goToA();
        }

        mtel.addData("Arm Power", robot.getArm().motor.getPower());
        mtel.addData("Arm Position", robot.getArm().motor.getCurrentPosition());
        mtel.addData("Arm Target", slidesPid.getTargetPosition());
        mtel.addData("CHub Current", robot.getControlHub().getCurrent(CurrentUnit.AMPS));
        mtel.addData("EXHub Current", robot.getExpansionHub().getCurrent(CurrentUnit.AMPS));
        mtel.update();
    }
}
