package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.control.Util.mtel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.robots.ChocolateRaisin;

@TeleOp
public class ChocolateRaisinTeleOp extends OpMode {
    ChocolateRaisin robot;
    PIDFController slidesPid;

    @Override
    public void init() {
        robot = new ChocolateRaisin(hardwareMap);
        slidesPid = new PIDFController(new PIDFController.PIDCoefficients(0.01, 0, 0));
    }

    @Override
    public void loop() {
        robot.getChassis().setDrivePowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.dpad_left) {
            slidesPid.setTargetPosition(robot.getSlides().UP_POS());
        } else if (gamepad1.dpad_right) {
            slidesPid.setTargetPosition(robot.getSlides().DOWN_POS());
        }

        if (gamepad1.dpad_up) {
            robot.getArm().motor.setPower(1);
        } else if (gamepad1.dpad_down) {
            robot.getArm().motor.setPower(-1);
        } else {
            robot.getArm().motor.setPower(0);
        }

        if (robot.getArm().motor.getPower() != 0) {
            robot.getSlides().stop();
        } else {
            robot.getSlides().setPower(slidesPid.update(robot.getSlides().getPosition()));
        }

        if (gamepad1.right_trigger > 0) {
            robot.getClaw().goToA();
        } else if (gamepad1.left_trigger > 0) {
            robot.getClaw().goToB();
        }

        mtel.addData("Arm Power", robot.getArm().motor.getPower());
        mtel.addData("Arm Position", robot.getArm().motor.getPosition());
        mtel.addData("Arm Target", slidesPid.targetPosition);
        mtel.addData("CHub Current", robot.getControlHub().getCurrent(CurrentUnit.AMPS));
        mtel.addData("EXHub Current", robot.getExpansionHub().getCurrent(CurrentUnit.AMPS));
        mtel.update();
    }
}
