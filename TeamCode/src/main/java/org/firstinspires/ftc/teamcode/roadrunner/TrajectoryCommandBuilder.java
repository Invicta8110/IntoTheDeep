package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.VelConstraint;

import dev.frozenmilk.mercurial.commands.Command;

public class TrajectoryCommandBuilder {
    @FunctionalInterface
    public interface TrajectoryCommandFactory {
        Command build(TimeTrajectory trajectory);
    }

    private final TrajectoryCommandFactory factory;
    private final TrajectoryBuilder builder;

    public TrajectoryCommandBuilder(TrajectoryCommandFactory factory, TrajectoryBuilder builder) {
        this.factory = factory;
        this.builder = builder;
    }


}
