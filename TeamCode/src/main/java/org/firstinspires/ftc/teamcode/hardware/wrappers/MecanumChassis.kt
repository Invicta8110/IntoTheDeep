package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive

class MecanumChassis @JvmOverloads constructor(hwMap: HardwareMap, pose: Pose2d = Pose2d(0.0, 0.0, 0.0)) : SparkFunOTOSDrive(hwMap, pose) {

    init {
        Motor.reverse(rightFront);
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
}