package org.firstinspires.ftc.teamcode.opmodes.rauton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

@Autonomous
public class RegularAuton extends LinearOpMode {
    private Motor fLeft, fRight, bLeft, bRight;

    @Override
    public void runOpMode() {

        fLeft = new Motor("fLeft",hardwareMap);
        fRight = new Motor("fRight",hardwareMap);
        bLeft = new Motor("bLeft",hardwareMap);
        bRight = new Motor("bRight",hardwareMap);

        bRight.reverse();
        fRight.reverse();

        waitForStart();
        fLeft.runToPosition((int)Util.CPR_435*5,1);
        fRight.runToPosition((int)Util.CPR_435*5,1);
        bLeft.runToPosition((int)Util.CPR_435*5,1);
        bRight.runToPosition((int)Util.CPR_435*5,1);

    }

}
