package org.firstinspires.ftc.teamcode.opmodes.rAuton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

@Config
@Autonomous
public class RegularAuton extends OpMode {
    private Motor fLeft, fRight, bLeft, bRight;
    public static PIDFController.PIDCoefficients coefs = new PIDFController.PIDCoefficients(0.01, 0, 0) ;
    private PIDFController fLController,bLController,fRController,bRController,armController;
    private MultipleTelemetry mtel;
    private Motor arm;

    @Override
    public void init() {
        arm = new Motor("arm",hardwareMap);
        arm.reverse();

        fLeft = new Motor("fLeft", hardwareMap);
        fRight = new Motor("fRight", hardwareMap);
        bLeft = new Motor("bLeft", hardwareMap);
        bRight = new Motor("bRight", hardwareMap);

        bRight.reverse();
        fRight.reverse();

        mtel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fLController = new PIDFController(coefs);
        fRController = new PIDFController(coefs);
        bLController = new PIDFController(coefs);
        bRController = new PIDFController(coefs);

//        armController = new PIDFController(coefs);
    }

    @Override
    public void loop() {
        double target = (int) (Util.CPI_435_104 * 12);

        fLController.setTargetPosition((int) (Util.CPI_435_104 * 12 * 1.5));
        bLController.setTargetPosition((int) (Util.CPI_435_104 * 12));
        fRController.setTargetPosition((int) (Util.CPI_435_104 * 12));
        bRController.setTargetPosition((int) (Util.CPI_435_104 * 12));
        double fLPower = fLController.update(fLeft.getPosition());
        double bLPower = fLController.update(bLeft.getPosition());
        double fRPower = fLController.update(fRight.getPosition());
        double bRPower = fLController.update(bRight.getPosition());


        fLeft.getInternal().setPower(fLPower);
        fRight.getInternal().setPower(fRPower);
        bLeft.getInternal().setPower(bLPower);
        bRight.getInternal().setPower(bRPower);

        mtel.addData("fLeft pos", fLeft.getPosition());
        mtel.addData("fLeft target", fLController.targetPosition);
        mtel.addData("fLeft power", fLPower);
        mtel.addData("error: ", fLController.targetPosition - fLeft.getPosition());

//        armController.setTargetPosition((int)(Util.CPI_435_104/2));
//        arm.getInternal().setPower(armController.update(arm.getPosition()));

        //arm.goUp();

        if(arm.getPosition()<1200)
            arm.getInternal().setPower(1);
        else if(arm.getPosition()>3000)
            arm.getInternal().setPower(0);
        else
            arm.getInternal().setPower(.5);
        mtel.addData("arm pos: ", arm.getPosition());
    }

}
