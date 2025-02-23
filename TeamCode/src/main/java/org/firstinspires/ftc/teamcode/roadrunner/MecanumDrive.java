package org.firstinspires.ftc.teamcode.roadrunner;

import static dev.frozenmilk.wavedash.Commands.DEFAULT_TRAJECTORY_PARAMS;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.DisplacementProfile;
import com.acmerobotics.roadrunner.DisplacementTrajectory;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.control.MecanumParams;
import org.firstinspires.ftc.teamcode.control.MecanumStatic;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.wavedash.Drive;
import dev.frozenmilk.wavedash.Localizer;
import dev.frozenmilk.wavedash.TrajectoryCommandBuilder;
import dev.frozenmilk.wavedash.messages.DriveCommandMessage;
import dev.frozenmilk.wavedash.messages.MecanumCommandMessage;
import dev.frozenmilk.wavedash.messages.MecanumLocalizerInputsMessage;
import dev.frozenmilk.wavedash.messages.PoseMessage;

@Config
public class MecanumDrive implements Drive {
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = MecanumStatic.getKinematics().forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(MecanumParams.getInPerTick()),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(MecanumParams.getInPerTick()),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(MecanumParams.getInPerTick()),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(MecanumParams.getInPerTick())
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta
            ));

            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (!module.getDeviceName().equals("Servo Hub 3"))
            { module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); }
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                MecanumParams.getLogoFacingDirection(), MecanumParams.getUsbFacingDirection()));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new PinpointLocalizer(hardwareMap, MecanumParams.getInPerTick(), pose);

        FlightRecorder.write("MECANUM_MecanumParams", MecanumParams.getToString());
    }


    @NonNull
    public Localizer getLocalizer() { return localizer; }
    
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public void setDrivePowersWithFF(PoseVelocity2dDual<Time> powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = MecanumStatic.getKinematics().inverse(powers);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(MecanumParams.getKS(),
                MecanumParams.getKV() / MecanumParams.getInPerTick(), MecanumParams.getKA() / MecanumParams.getInPerTick());
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    @Override
    public void setDrivePowersWithFF(@NonNull PoseVelocity2d powers) {
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 1));
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        
        
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * Follow a trajectory.
     * @param trajectory trajectory to follow
     * @param t time to follow in seconds
     * @return whether the trajectory has been completed
     */
    public boolean followTrajectory(TimeTrajectory trajectory, double t) {
        Log.e("WAVEDASH", String.format("following trajectory %s at time %f", trajectory, t));

        if (t >= trajectory.duration) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            return true;
        }

        Pose2dDual<Time> txWorldTarget = trajectory.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                MecanumParams.getAxialGain(), MecanumParams.getLateralGain(), MecanumParams.getHeadingGain(),
                MecanumParams.getAxialVelGain(), MecanumParams.getLateralVelGain(), MecanumParams.getHeadingVelGain()
        )
                .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = MecanumStatic.getKinematics().inverse(command);
        double voltage = voltageSensor.getVoltage();

        double leftFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.leftFront) / voltage;
        double leftBackPower = MecanumStatic.getFeedforward().compute(wheelVels.leftBack) / voltage;
        double rightBackPower = MecanumStatic.getFeedforward().compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        return false;
    }

    /**
     * Follow a trajectory.
     * @param trajectory trajectory to follow
     * @param t time to follow in seconds
     * @return whether the trajectory has been completed
     **/
    public boolean followTrajectory(@NonNull Trajectory trajectory, double t) {
        return followTrajectory(new TimeTrajectory(trajectory), t);
    }

    @Override
    public boolean turn(TimeTurn turn, double t) {
        if (t >= turn.duration) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            return true;
        }

        Pose2dDual<Time> txWorldTarget = turn.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = MecanumStatic.getController().compute(txWorldTarget, localizer.getPose(), robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = MecanumStatic.getKinematics().inverse(command);
        double voltage = voltageSensor.getVoltage();

        double leftFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.leftFront) / voltage;
        double leftBackPower = MecanumStatic.getFeedforward().compute(wheelVels.leftBack) / voltage;
        double rightBackPower = MecanumStatic.getFeedforward().compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(MecanumStatic.getFeedforward().compute(wheelVels.leftFront) / voltage);
        leftBack.setPower(MecanumStatic.getFeedforward().compute(wheelVels.leftBack) / voltage);
        rightBack.setPower(MecanumStatic.getFeedforward().compute(wheelVels.rightBack) / voltage);
        rightFront.setPower(MecanumStatic.getFeedforward().compute(wheelVels.rightFront) / voltage);

        return false;
    }

    @NonNull
    public Command followTrajectoryCommand(@NonNull TimeTrajectory trajectory) {

        return new FollowTrajectoryCommand(trajectory);
    }

    @NonNull
    @Override
    public TrajectoryCommandBuilder commandBuilder(@NonNull Pose2d beginPose) {
        return new TrajectoryCommandBuilder(
                this::turnCommand,
                FollowTrajectoryCommand::new,
                DEFAULT_TRAJECTORY_PARAMS,
                beginPose,
                0.0,
                MecanumStatic.getDefaultTurnConstraints(),
                MecanumStatic.getDefaultVelConstraint(),
                MecanumStatic.getDefaultAccelConstraint()
        );
    }

    @NonNull
    public TrajectoryActionBuilder actionBuilder(@NonNull Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                DEFAULT_TRAJECTORY_PARAMS,
                beginPose,
                0.0,
                MecanumStatic.getDefaultTurnConstraints(),
                MecanumStatic.getDefaultVelConstraint(),
                MecanumStatic.getDefaultAccelConstraint()
        );
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    MecanumParams.getAxialGain(), MecanumParams.getLateralGain(), MecanumParams.getHeadingGain(),
                    MecanumParams.getAxialVelGain(), MecanumParams.getLateralVelGain(), MecanumParams.getHeadingVelGain()
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = MecanumStatic.getKinematics().inverse(command);
            double voltage = voltageSensor.getVoltage();

            double leftFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.leftFront) / voltage;
            double leftBackPower = MecanumStatic.getFeedforward().compute(wheelVels.leftBack) / voltage;
            double rightBackPower = MecanumStatic.getFeedforward().compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = MecanumStatic.getController().compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = MecanumStatic.getKinematics().inverse(command);
            double voltage = voltageSensor.getVoltage();

            double leftFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.leftFront) / voltage;
            double leftBackPower = MecanumStatic.getFeedforward().compute(wheelVels.leftBack) / voltage;
            double rightBackPower = MecanumStatic.getFeedforward().compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = MecanumStatic.getFeedforward().compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(MecanumStatic.getFeedforward().compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(MecanumStatic.getFeedforward().compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(MecanumStatic.getFeedforward().compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(MecanumStatic.getFeedforward().compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    private class FollowTrajectoryCommand implements Command {
        final Set<Object> requirements;
        final Set<Wrapper.OpModeState> runStates;
        private final TimeTrajectory trajectory;

        ElapsedTime timer;
        boolean finished;

        public FollowTrajectoryCommand(TimeTrajectory trajectory) {
            this.trajectory = trajectory;
            requirements = new HashSet<>();
            runStates = Set.of(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT);
            timer = new ElapsedTime();
            finished = false;
            ;
            Log.e("WAVEDASH", "Initializing trajectory");
        }

        @NonNull
        @Override
        public Set<Wrapper.OpModeState> getRunStates() {
            return runStates;
        }

        @NonNull
        @Override
        public Set<Object> getRequirements() {
            return requirements;
        }

        @Override
        public boolean finished() {
            return finished;
        }

        @Override
        public void end(boolean b) {
            MecanumDrive.this.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));

        }

        @Override
        public void execute() {
            finished = MecanumDrive.this.followTrajectory(trajectory, timer.seconds());
        }

        @Override
        public void initialise() {
            Log.e("WAVEDASH", "Starting trajectory");
            timer.reset();
        }
    }

    class FollowPathAction implements Action {
        private final DisplacementTrajectory traj;
        private final PosePath path;
        private final DisplacementProfile profile;
        private double disp = 0.0;

        public FollowPathAction(DisplacementTrajectory traj) {
            this.traj = traj;
            this.path = traj.path;
            this.profile = traj.profile;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            PoseVelocity2d robotVel = updatePoseEstimate();
            disp = traj.project(localizer.getPose().position, disp);

            if (disp >= path.length() ||
                    (traj.get(traj.length()).position.value().minus(localizer.getPose().position)).norm() < 2) {
                setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                return false;
            }

            PoseVelocity2dDual<Time> command = MecanumStatic.getController().compute(
                    traj.get(disp),
                    localizer.getPose(),
                    robotVel
            );

            setDrivePowersWithFF(command);

            return true;
        }
    }


}
