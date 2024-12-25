package org.firstinspires.ftc.teamcode.opmodes.old.rauton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

import page.j5155.expressway.ftc.motion.PIDFController;

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
        double fLPower = fLController.update(fLeft.getCurrentPosition());
        double bLPower = fLController.update(bLeft.getCurrentPosition());
        double fRPower = fLController.update(fRight.getCurrentPosition());
        double bRPower = fLController.update(bRight.getCurrentPosition());


        fLeft.setPower(fLPower);
        fRight.setPower(fRPower);
        bLeft.setPower(bLPower);
        bRight.setPower(bRPower);

        mtel.addData("fLeft pos", fLeft.getCurrentPosition());
        mtel.addData("fLeft target", fLController.getTargetPosition());
        mtel.addData("fLeft power", fLPower);
        mtel.addData("error: ", fLController.getTargetPosition() - fLeft.getCurrentPosition());

//        armController.setTargetPosition((int)(Util.CPI_435_104/2));
//        arm.getInternal().setPower(armController.update(arm.getCurrentPosition()));

        //arm.goUp();

        if(arm.getCurrentPosition()<1200)
            arm.setPower(1);
        else if(arm.getCurrentPosition()>3000)
            arm.setPower(0);
        else
            arm.setPower(.5);
        mtel.addData("arm pos: ", arm.getCurrentPosition());
    }

}
