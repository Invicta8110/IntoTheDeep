package org.firstinspires.ftc.teamcode.opmodes.examples;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.control.ActionOpMode;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

public class CsTeleopExample extends ActionOpMode {
    MecanumDrive drive;
    LinearSlides slides;
    TwoPointServo claw;
    SDKGamepad gamepad;
    Motor intake;

    @Override
    public void init() {
        gamepad = (SDKGamepad) Pasteurized.gamepad1();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        slides = new LinearSlides(hardwareMap);
        claw = new TwoPointServo(
                hardwareMap.get(ServoImplEx.class, "claw"),
                1, 0.5);
        intake = new Motor(hardwareMap.get(DcMotorEx.class, "intake"));

        claw.runToA().run(new TelemetryPacket());
    }

    @Override
    public void loop() {
        if (gamepad.a().onTrue()) {
            add(new InstantAction(() -> intake.getInternal().setPower(1)));
        } else if (gamepad.b().onTrue()) {
            add(new InstantAction(() -> intake.getInternal().setPower(0)));
        }

        if (gamepad.leftTrigger().state() > 0) {
            add(depositAction());
        } else if (gamepad.rightTrigger().state() > 0) {
            add(claw.runAction());
        }

        if (gamepad.dpadUp().onTrue()) {
            add(slides.powerAction(1));
        } else if (gamepad.dpadUp().onFalse()) {
            add(slides.powerAction(0));
        } else if (gamepad.dpadDown().onTrue()) {
            add(slides.powerAction(-1));
        } else if (gamepad.dpadDown().onFalse()) {
            add(slides.powerAction(0));
        }

        telemetry.addData(
                "Slide Current",
                slides.get(0).getCurrent()
        );
        telemetry.addData("Slide Position", slides.get(0).getPosition());
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Pose:", drive.pose);
        telemetry.update();

        super.loop();
    }

    public SequentialAction depositAction() {
        return new SequentialAction(
                slides.powerAction(1),
                new SleepAction(1),
                slides.powerAction(0),
                claw.runToA(),
                new SleepAction(1),
                slides.powerAction(-1),
                new SleepAction(1),
                slides.powerAction(0)
        );
    }
}