package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive
import page.j5155.expressway.ftc.motion.PIDFController
import page.j5155.expressway.ftc.motion.PIDToPoint

class MecanumChassis @JvmOverloads constructor(hwMap: HardwareMap, pose: Pose2d = Pose2d(0.0, 0.0, 0.0)) : SparkFunOTOSDrive(hwMap, pose) {

    @JvmField val RR_PARAMS = MecanumDrive.PARAMS

    init {
        Motor.reverse(rightFront)
        Motor.reverse(rightBack)
    }

    fun setDrivePowers(x: Double, y: Double, heading: Double) = setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))

    fun moveToPointAction(x: Double, y: Double): Action = actionBuilder(pose)
        .splineTo(Vector2d(x, y), pose.heading)
        .build()

    fun turnAction(angle: Double): Action  = actionBuilder(pose)
        .turnTo(angle)
        .build()


    fun followTrajectoryAction(trajectory: List<Pose2d>): Action {
        var traj = actionBuilder(pose)
        for (pose in trajectory) {
            traj = traj.splineTo(pose.position, pose.heading)
        }
        return traj.build()
    }

    fun drivePowerAction(x: Double, y: Double, heading: Double): Action
        = Action {
            setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))
            false
        }

    /*
        public PIDToPoint pidToPointAction(Pose2d target) {
        return new PIDToPoint(
                (this::getPose)
                (this::updatePoseEstimate) // updatePoseEstimate, in addition to updating the pose property, returns the velocity of the robot
                target, // the target pose
                (this::setDrivePowers), // setDrivePowers uses inverse kinematics to set the powers of the drivetrain motors
                new PIDFController.PIDCoefficients(PARAMS.axialGain, 0, PARAMS.axialVelGain), // the axial PID coefficients
                new PIDFController.PIDCoefficients(PARAMS.lateralGain, 0, PARAMS.lateralVelGain), // the lateral PID coefficients
                new PIDFController.PIDCoefficients(PARAMS.headingGain, 0, PARAMS.headingVelGain) // the heading PID coefficients
        );
    }
     */

    fun moveToPoint(target: Pose2d): PIDToPoint = PIDToPoint(
        this::pose,
        this::updatePoseEstimate,
        target,
        this::setDrivePowers,
        PIDFController.PIDCoefficients(RR_PARAMS.axialGain, 0.0, RR_PARAMS.axialVelGain), // the axial PID coefficients
        PIDFController.PIDCoefficients(RR_PARAMS.lateralGain, 0.0, RR_PARAMS.lateralVelGain), // the lateral PID coefficients
        PIDFController.PIDCoefficients(RR_PARAMS.headingGain, 0.0, RR_PARAMS.headingVelGain) // the heading PID coefficients
    )
}