package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.range
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.ceil
import kotlin.math.max

class FollowTrajectoryAsPathAction(
    t: TimeTrajectory,
    axialGain: Double,
    lateralGain: Double,
    headingGain: Double,
    axialVelGain: Double,
    lateralVelGain: Double,
    headingVelGain: Double,
    val powerUpdater: Consumer<PoseVelocity2dDual<Time>>,
    val velocityEstimater: Supplier<PoseVelocity2d>,
    val positionEstimater: Supplier<Pose2d>
) : Action {
    val dispTraj: DisplacementTrajectory = DisplacementTrajectory(t.path, t.profile.dispProfile)
    val contr: HolonomicController = HolonomicController( // PD to point/velocity controller
        axialGain,
        lateralGain,
        headingGain,
        axialVelGain,
        lateralVelGain,
        headingVelGain
    )

    private val xPoints: DoubleArray
    private val yPoints: DoubleArray
    var disp: Double // displacement; target distance traveled in path

    // only used for recording what end time should be
    // to avoid the dreaded wiggle
    val trajectoryRunningTime: ElapsedTime = ElapsedTime()
    var targetTimeSeconds: Double
    var initialized: Boolean = false

    init {
        disp = 0.0

        targetTimeSeconds = t.duration


        // ONLY USED FOR PREVIEW
        val disps = range( // returns evenly spaced values
            0.0,  // between 0 and the length of the path
            dispTraj.path.length(),
            max(
                2.0,  // minimum 2
                ceil(dispTraj.path.length() / 2) // max total of half the length of the path
            ).toInt()
        )

        // so really make 1 sample every 2 inches (I think)

        // and then convert them into lists of doubles of x and y so they can be shown on dash
        xPoints = DoubleArray(disps.size)
        yPoints = DoubleArray(disps.size)
        for (i in disps.indices) {
            val p = t.path[disps[i], 1].value()
            xPoints[i] = p.position.x
            yPoints[i] = p.position.y
        }
    }

    override fun run(p: TelemetryPacket): Boolean {
        // needs to only run once
        // idk if this is the most elegant solution
        if (!initialized) {
            trajectoryRunningTime.reset()
            initialized = true
        }


        val robotVelRobot = velocityEstimater.get()
        val robotPose = positionEstimater.get()

        // find the closest position on the path to the robot's current position
        // (using binary search? I think? project function is hard to understand)
        // where "position on the path" is represent as disp or distance into the path
        // so like for a 10 inch long path, if disp was 5 it would be halfway along the path
        disp = dispTraj.project(robotPose.position, disp)

        // check if the trajectory should end
        // this logic is pretty much made up and doesnt really make sense
        // and it wiggles occasionally
        // but it does usually work

        // if robot within 2 in of end pose
        if (dispTraj[dispTraj.length()].position.value().minus(robotPose.position)
                .norm() < 2 // or the closest position on the path is less then 2 inches away from the end of the path
            || (disp + 2) >= dispTraj.length() // or the trajectory has been running for 1 second more then it's suppposed to (this 1 second is weird)
            || trajectoryRunningTime.seconds() >= targetTimeSeconds + 1
        ) {
            // stop all the motors

            powerUpdater.accept(PoseVelocity2dDual.constant(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1))

            // end the action
            return false
        }

        // ok so the trajectory shouldn't end yet

        // find the target pose and vel of the closest point on the path
        val poseTarget = dispTraj[disp]

        // calculate the command based on PD on the target pose and vel
        val cmd = contr.compute(poseTarget, robotPose, robotVelRobot)

        // send the command to the motors
        powerUpdater.accept(cmd)

        // log target to rr logs
        write("TARGET_POSE", PoseMessage(poseTarget.value()))

        // show dash data
        p.put("x", robotPose.position.x)
        p.put("y", robotPose.position.y)
        p.put("heading (deg)", Math.toDegrees(robotPose.heading.log()))

        val error = poseTarget.value().minusExp(robotPose)
        p.put("xError", error.position.x)
        p.put("yError", error.position.y)
        p.put("headingError (deg)", Math.toDegrees(error.heading.log()))

        // only draw when active; only one drive action should be active at a time
        val c = p.fieldOverlay()

        c.setStroke("#4CAF50")
        Drawing.drawRobot(c, poseTarget.value())

        c.setStroke("#3F51B5")
        Drawing.drawRobot(c, robotPose)

        c.setStroke("#4CAF50FF")
        c.setStrokeWidth(1)
        c.strokePolyline(xPoints, yPoints)

        // continue running the action
        return true
    }
}