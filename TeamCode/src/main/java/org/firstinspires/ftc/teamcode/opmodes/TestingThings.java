package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class TestingThings extends LinearOpMode {
    public void runOpMode() {
        Motor m = new Motor("motor1", hardwareMap);

        waitForStart();

        m.getDcMotorEx().setPower(1);
        m.invoke().setPower(1);


    }
}
