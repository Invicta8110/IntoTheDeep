package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import dev.frozenmilk.wavedash.GoBildaPinpointDriver;

public class PinpointViewImpl implements PinpointView {

    GoBildaPinpointDriver driver;
    GoBildaPinpointDriver.EncoderDirection parDirection;
    GoBildaPinpointDriver.EncoderDirection perpDirection;

    public PinpointViewImpl(PinpointLocalizer pl) {
        this.driver = pl.driver;
        this.parDirection = pl.initialParDirection;
        this.perpDirection = pl.initialPerpDirection;
    }

    @Override
    public void update() {
        driver.update();
    }

    @Override
    public int getParEncoderPosition() {
        return driver.getEncoderX();
    }

    @Override
    public int getPerpEncoderPosition() {
        return driver.getEncoderY();
    }

    @Override
    public float getHeadingVelocity() {
        return -(float) driver.getHeadingVelocity();
    }

    @Override
    public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
        parDirection = direction == DcMotorSimple.Direction.FORWARD ?
                GoBildaPinpointDriver.EncoderDirection.FORWARD :
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        driver.setEncoderDirections(parDirection, perpDirection);
    }

    @Override
    public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
        perpDirection = direction == DcMotorSimple.Direction.FORWARD ?
                GoBildaPinpointDriver.EncoderDirection.FORWARD :
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        driver.setEncoderDirections(parDirection, perpDirection);
    }
}