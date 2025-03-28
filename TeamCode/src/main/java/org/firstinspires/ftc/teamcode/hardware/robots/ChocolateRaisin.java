package org.firstinspires.ftc.teamcode.hardware.robots;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.Util;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.MotorArm;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlidesManual;
import org.firstinspires.ftc.teamcode.hardware.wrappers.TwoPointServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.OTOSLocalizer;

import java.util.List;

public class ChocolateRaisin {
    private final MecanumChassis chassis;
    private final LinearSlidesManual slides;
    private final MotorArm motorArm;
    private final TwoPointServo claw;
    private LynxModule controlHub, expansionHub;
    private SparkFunOTOS otos;

    public ChocolateRaisin(HardwareMap hwMap, Pose2d pose) {
        chassis = new MecanumChassis(hwMap, pose);
        slides = new LinearSlidesManual(new Motor("slides", hwMap));
        motorArm = new MotorArm(new Motor("armRight", hwMap));
        claw = new TwoPointServo("claw", hwMap, .75, 1);
        otos = ((OTOSLocalizer) chassis.getLocalizer()).otos;

        List<LynxModule> hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            if (module.isParent() && controlHub == null) {
                controlHub = module;
            } else if (expansionHub == null) {
                expansionHub = module;
            }
        }
    }

    public ChocolateRaisin(HardwareMap hwMap) {
        this(hwMap, new Pose2d(0, 0, 0));
    }

    public MecanumChassis getChassis() {
        return chassis;
    }

    public LinearSlidesManual getSlides() {
        return slides;
    }

    public MotorArm getArm() {
        return motorArm;
    }

    public TwoPointServo getClaw() {
        return claw;
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public LynxModule getExpansionHub() {
        return expansionHub;
    }

    public SparkFunOTOS getOTOS() { return otos; }

    public Pose2d getPose() {
        return chassis.getLocalizer().getPose();
    }

    private void configureOtos() {
        Util.mtel.addLine("Configuring OTOS...");
        Util.mtel.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        otos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        Util.mtel.addLine("OTOS configured! Press start to get position data!");
        Util.mtel.update();
    }
}
