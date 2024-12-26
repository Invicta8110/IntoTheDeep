package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DisplacementTrajectory;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

import java.util.List;

// Alternate trajectory follower for roadrunner using displacement trajectories (distance instead of time)
// to use, put at the bottom of RR 1.0's MecanumDrive file, and change actionBuilder to use it instead of FollowTrajectoryAction
// Created by j5155 from team 12087 based on https://rr.brott.dev/docs/v1-0/guides/path-following/
// Licensed under the BSD 3-Clause Clear License
// If you use this, I would love to know how it goes/what issues you encounter, I'm @j5155 on discord
public final class FollowTrajectoryAsPathAction implements Action {
    public final DisplacementTrajectory dispTraj;
    public final HolonomicController contr;

    private final double[] xPoints, yPoints;
    double disp; // displacement; target distance traveled in path

    // only used for recording what end time should be
    // to avoid the dreaded wiggle
    public final ElapsedTime trajectoryRunningTime = new ElapsedTime();
    public double targetTimeSeconds;
    private MecanumDrive robot;
    boolean initialized = false;

    public FollowTrajectoryAsPathAction(TimeTrajectory t, MecanumDrive robot) {
        this.robot = robot;

        dispTraj = new DisplacementTrajectory(t.path, t.profile.dispProfile);
        contr = new HolonomicController( // PD to point/velocity controller
                PARAMS.axialGain,
                PARAMS.lateralGain,
                PARAMS.headingGain,
                PARAMS.axialVelGain,
                PARAMS.lateralVelGain,
                PARAMS.headingVelGain);
        disp = 0;

        targetTimeSeconds = t.duration;


        // ONLY USED FOR PREVIEW
        List<Double> disps = com.acmerobotics.roadrunner.Math.range( // returns evenly spaced values
                0,  // between 0 and the length of the path
                dispTraj.path.length(),
                Math.max(2, // minimum 2
                        (int) Math.ceil(dispTraj.path.length() / 2) // max total of half the length of the path
                ));
        // so really make 1 sample every 2 inches (I think)

        // and then convert them into lists of doubles of x and y so they can be shown on dash
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
        // needs to only run once
        // idk if this is the most elegant solution
        if (!initialized) {
            trajectoryRunningTime.reset();
            initialized = true;
        }


        PoseVelocity2d robotVelRobot = robot.updatePoseEstimate();

        // find the closest position on the path to the robot's current position
        // (using binary search? I think? project function is hard to understand)
        // where "position on the path" is represent as disp or distance into the path
        // so like for a 10 inch long path, if disp was 5 it would be halfway along the path
        disp = dispTraj.project(robot.pose.position, disp);

        // check if the trajectory should end
        // this logic is pretty much made up and doesnt really make sense
        // and it wiggles occasionally
        // but it does usually work

        // if robot within 2 in of end pose
        if (dispTraj.get(dispTraj.length()).position.value().minus(robot.pose.position).norm() < 2
                // or the closest position on the path is less then 2 inches away from the end of the path
                || (disp + 2) >= dispTraj.length()
                // or the trajectory has been running for 1 second more then it's suppposed to (this 1 second is weird)
                || trajectoryRunningTime.seconds() >= targetTimeSeconds + 1) {

            // stop all the motors
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);

            // end the action
            return false;
        }

        // ok so the trajectory shouldn't end yet

        // find the target pose and vel of the closest point on the path
        Pose2dDual<Time> poseTarget = dispTraj.get(disp);

        // calculate the command based on PD on the target pose and vel
        PoseVelocity2dDual<Time> cmd = contr.compute(poseTarget, robot.pose, robotVelRobot);

        // convert it into wheel velocities with inverse kinematics
        MecanumKinematics.WheelVelocities<Time> wheelVels = robot.kinematics.inverse(cmd);
        // find voltage for voltage compensation
        double voltage = robot.voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, 0); // kA 0; ignore acceleration

        // calculate the volts to send to each wheel based on the target velocity for the wheel
        // divide it by the current voltage to get the power from 0-1
        robot.leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
        robot.leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
        robot.rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
        robot.rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

        // log target to rr logs
        FlightRecorder.write("TARGET_POSE", new PoseMessage(poseTarget.value()));

        // show dash data
        p.put("x", robot.pose.position.x);
        p.put("y", robot.pose.position.y);
        p.put("heading (deg)", Math.toDegrees(robot.pose.heading.log()));

        Pose2d error = poseTarget.value().minusExp(robot.pose);
        p.put("xError", error.position.x);
        p.put("yError", error.position.y);
        p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

        // only draw when active; only one drive action should be active at a time
        Canvas c = p.fieldOverlay();
        robot.drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, poseTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, robot.pose);

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        // continue running the action
        return true;
    }
}