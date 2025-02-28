package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import dev.frozenmilk.wavedash.Localizer;
import dev.frozenmilk.wavedash.GoBildaPinpointDriver;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        // mmPerTick can either be tuned manually using PinpointForwardPushTest,
        // or by dividing the the circumference of your odometry wheels **in mm** by the number of ticks per revolution
        public double mmPerTick = 1.0 / 13.26291192;

        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        driver.setEncoderResolution(1 / PARAMS.mmPerTick);
        driver.setOffsets(PARAMS.mmPerTick * PARAMS.parYTicks, PARAMS.mmPerTick * PARAMS.perpXTicks);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX() / 25.4, driver.getPosY() / 25.4, driver.getHeading());
            Vector2d worldVelocity = new Vector2d(driver.getVelX() / 25.4, driver.getVelY() / 25.4);
            Vector2d robotVelocity = Rotation2d.fromDouble(-driver.getHeading()).times(worldVelocity);
            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity());
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    @NonNull
    public PinpointView getView() {
        return new PinpointViewImpl();
    }

    private class PinpointViewImpl implements PinpointView {
        PoseVelocity2d currentVel = new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);

        @NonNull
        @Override
        public DcMotorSimple.Direction getParDirection() {
            if (initialParDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD) {
                return DcMotorSimple.Direction.FORWARD;
            } else {
                return DcMotorSimple.Direction.REVERSE;
            }
        }

        @Override
        public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
            if (direction == DcMotorSimple.Direction.FORWARD) {
                driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, initialPerpDirection);
            } else {
                driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, initialPerpDirection);
            }
        }

        @NonNull
        @Override
        public DcMotorSimple.Direction getPerpDirection() {
            if (initialPerpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD) {
                return DcMotorSimple.Direction.FORWARD;
            } else {
                return DcMotorSimple.Direction.REVERSE;
            }
        }

        @Override
        public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
            if (direction == DcMotorSimple.Direction.FORWARD) {
                driver.setEncoderDirections(initialParDirection, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            } else {
                driver.setEncoderDirections(initialParDirection, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
        }

        @NonNull
        @Override
        public Pose2d getPose() {
            return PinpointLocalizer.this.getPose();
        }

        @Override
        public void setPose(@NonNull Pose2d pose2d) {
            PinpointLocalizer.this.setPose(pose2d);
        }

        @NonNull
        @Override
        public PoseVelocity2d getVel() {
            return currentVel;
        }

        @Override
        public void setVel(@NonNull PoseVelocity2d poseVelocity2d) {
            throw new UnsupportedOperationException("PinpointLocalizer does not support setting velocity");
        }

        @Override
        public void update() {
            currentVel = PinpointLocalizer.this.update();
        }
    }
}