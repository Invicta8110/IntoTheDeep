package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Line
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.wavedash.DEFAULT_TRAJECTORY_PARAMS
import dev.frozenmilk.wavedash.Drive
import dev.frozenmilk.wavedash.Localizer
import dev.frozenmilk.wavedash.TrajectoryCommandBuilder
import dev.frozenmilk.wavedash.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.control.MecanumParams
import org.firstinspires.ftc.teamcode.control.MecanumParams.inPerTick
import org.firstinspires.ftc.teamcode.control.MecanumStatic
import org.firstinspires.ftc.teamcode.control.MecanumStatic.feedforward
import org.firstinspires.ftc.teamcode.control.PIDController
import org.firstinspires.ftc.teamcode.control.followPathCommand
import org.firstinspires.ftc.teamcode.control.project
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage


class MecanumChassis @JvmOverloads constructor(
    hwMap: HardwareMap,
    pose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) : MecanumDrive(hwMap, pose), Drive {
    var currentPosition by this.mdLocalizer::pose
    override val localizer: Localizer = super.mdLocalizer

    val feedforward = MotorFeedforward(
        MECANUM_PARAMS.kS,
        MECANUM_PARAMS.kV / MECANUM_PARAMS.inPerTick,
        MECANUM_PARAMS.kA / MECANUM_PARAMS.inPerTick
    )

    override fun commandBuilder(beginPose: Pose2d): TrajectoryCommandBuilder {
        return TrajectoryCommandBuilder(
            this::turnCommand,
            this::followTrajectoryCommand,
            DEFAULT_TRAJECTORY_PARAMS,
            beginPose,
            0.0,
            defaultTurnConstraints,
            defaultVelConstraint,
            defaultAccelConstraint
        )
    }

    override fun followTrajectory(trajectory: TimeTrajectory, t: Double): Boolean {
        if (t >= trajectory.duration) {
            leftFront.power = 0.0
            leftBack.power = 0.0
            rightBack.power = 0.0
            rightFront.power = 0.0

            return false
        }

        val txWorldTarget: Pose2dDual<Time> = trajectory[t]
        targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

        val robotVelRobot = updatePoseEstimate()

        val command = controller.compute(txWorldTarget, mdLocalizer.pose, robotVelRobot)
        driveCommandWriter.write(DriveCommandMessage(command))

        val wheelVels = kinematics.inverse(command)
        val voltage = voltageSensor.voltage

        val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
        val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
        val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
        val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage

        mecanumCommandWriter.write(
            org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            )
        )

        leftFront.power = leftFrontPower
        leftBack.power = leftBackPower
        rightBack.power = rightBackPower
        rightFront.power = rightFrontPower

        return true
    }

    fun setDrivePowers(x: Double, y: Double, heading: Double) = setDrivePowers(PoseVelocity2d(Vector2d(x, y), heading))

    fun setDrivePowers(vector: Vector2d, heading: Double) = setDrivePowers(PoseVelocity2d(vector, heading))

    fun drivePowerAction(pose: PoseVelocity2d): Action = InstantAction { setDrivePowers(pose) }

    fun drivePowerAction(x: Double, y: Double, heading: Double): Action
        = InstantAction { setDrivePowers(x, y, heading) }

    fun trajectoryBuilder(beginPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(
            DEFAULT_TRAJECTORY_PARAMS,
            beginPose,
            0.0,
            defaultVelConstraint,
            defaultAccelConstraint
        )
    }

    fun setPowersWithDirection(target: Pose2dDual<Time>) {
        val robotVel: PoseVelocity2d = this.updatePoseEstimate()

        val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
            targetPose = target,
            actualPose = this.mdLocalizer.pose,
            actualVelActual = robotVel
        )

        val wheelVels = kinematics.inverse<Time>(command)
        val voltage = voltageSensor.voltage

        leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
        leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
        rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
        rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage
    }
    fun setPowersWithDirection(target: Pose2d) = setPowersWithDirection(Pose2dDual.constant(target, 3))

    fun moveToPoint(target: Pose2d): Command {
        return Lambda("Move to $target")
            .setExecute {
                setPowersWithDirection(target)
            }.setFinish {
                val error: Twist2d = target - this.mdLocalizer.pose
                error.line.norm() < 1.0 && error.angle < Math.PI/3
            }.setEnd {
                this.setDrivePowers(0.0, 0.0, 0.0)
            }
    }

    fun turnTo(target: Double): Command {
        return Lambda("Turn to $target")
            .setExecute {
                setPowersWithDirection(Pose2d(0.0, 0.0, target))
            }.setFinish {
                val error: Double = target - this.mdLocalizer.pose.heading.toDouble()
                error < Math.PI/8
            }
    }

    fun strafeTo(target: Vector2d): Command {
        return Lambda("strafe-to-$target")
            .setExecute {
                val robotVel = updatePoseEstimate()
                val error = target - currentPosition.position

                setDrivePowers(PoseVelocity2d(error, robotVel.angVel))
            }.setFinish {
                val error: Vector2d = target - this.mdLocalizer.pose.position
                error.norm() < 4.0
            }
    }

    fun pathBuilder(beginPose: Pose2d) =
        TrajectoryCommandBuilder(
            this::turnCommand,
            this::followPathCommand,
            DEFAULT_TRAJECTORY_PARAMS,
            beginPose,
            0.0,
            defaultTurnConstraints,
            defaultVelConstraint,
            defaultAccelConstraint
        )

    fun pidToPoint(target: Vector2d): Command {
        val xController = PIDController(PIDController.PIDCoefficients(1.0, 0.0, 0.0))
        val yController = PIDController(PIDController.PIDCoefficients(1.0, 0.0, 0.0))

        return Lambda("pid-to-$target")
            .setInit {
                xController.targetPosition = target.x / MECANUM_PARAMS.inPerTick
                yController.targetPosition = target.y / MECANUM_PARAMS.inPerTick
            }.setExecute {
                updatePoseEstimate()

                val newX = xController.update(currentPosition.position.x / MECANUM_PARAMS.inPerTick) * MECANUM_PARAMS.inPerTick
                val newY = yController.update(currentPosition.position.y / MECANUM_PARAMS.inPerTick) * MECANUM_PARAMS.inPerTick

                setDrivePowersWithFF(PoseVelocity2d(Vector2d(newX, newY), 0.0))
            }.setEnd {
                setDrivePowers(0.0, 0.0, 0.0)
            }.setFinish {
                val error = target - currentPosition.position

                error.norm() < 1.0
            }
    }

    fun followTurnCommand(turn: TimeTurn) : Command {
        val timer = ElapsedTime()
        return Lambda("follow-turn")
            .setInit {
                timer.reset()
            }
            .setExecute {
                val target = turn[timer.seconds()]
                val robotVel: PoseVelocity2d = this.updatePoseEstimate()
                val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
                    targetPose = target,
                    actualPose = currentPosition,
                    actualVelActual = robotVel
                )
                this.setDrivePowers(command.value())
            }.setEnd {
                setDrivePowers(0.0, 0.0, 0.0)
            }.setFinish {
                timer.seconds() >= turn.duration
            }
    }

    override fun followTrajectoryCommand(trajectory: TimeTrajectory): Command {
        val timer = ElapsedTime()

        return Lambda("follow-trajectory")
            .setInit {
                timer.reset()
            }
            .setExecute {
                val target = trajectory[timer.seconds()]
                val robotVel: PoseVelocity2d = this.updatePoseEstimate()
                val command: PoseVelocity2dDual<Time> = MecanumStatic.controller.compute(
                    targetPose = target,
                    actualPose = currentPosition,
                    actualVelActual = robotVel
                )
                this.setDrivePowers(command.value())
            }.setEnd {
                setDrivePowers(0.0, 0.0, 0.0)
            }.setFinish {
                timer.seconds() >= trajectory.duration
            }
    }

    fun setDrivePowersWithFF(powers: PoseVelocity2dDual<Time>) {
        val wheelVels = kinematics.inverse<Time>(powers)
        val voltage = voltageSensor.voltage

        val feedforward = MotorFeedforward(
            MECANUM_PARAMS.kS,
            MECANUM_PARAMS.kV / MECANUM_PARAMS.inPerTick, MECANUM_PARAMS.kA / MECANUM_PARAMS.inPerTick
        )

        val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
        val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
        val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
        val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage

        mecanumCommandWriter.write(
            MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            )
        )

        leftFront.power = leftFrontPower
        leftBack.power = leftBackPower
        rightBack.power = rightBackPower
        rightFront.power = rightFrontPower
    }
    override fun setDrivePowersWithFF(powers: PoseVelocity2d) =
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 3))

    override fun turn(turn: TimeTurn, t: Double): Boolean {
        if (t >= turn.duration) {
            leftFront.power = 0.0
            leftBack.power = 0.0
            rightBack.power = 0.0
            rightFront.power = 0.0

            return true
        }

        val txWorldTarget = turn[t]
        targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

        val robotVelRobot = updatePoseEstimate()

        val command = controller.compute(txWorldTarget, mdLocalizer.pose, robotVelRobot)
        driveCommandWriter.write(DriveCommandMessage(command))

        val wheelVels = MecanumStatic.kinematics.inverse(command)
        val voltage = voltageSensor.voltage

        val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
        val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
        val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
        val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
        mecanumCommandWriter.write(
            MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            )
        )

        leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
        leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
        rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
        rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

        return false
    }

    fun lineTo(target: Vector2d): Command {
        val line = Line(currentPosition.position, target)
        val timer = ElapsedTime()
        var disp = 0.0

        return Lambda("line-to-$target")
            .setInit { timer.reset() }
            .setExecute {
                val robotVel = updatePoseEstimate()
                disp = line.project(currentPosition.position, disp)
                val command = controller.compute(
                    targetPose = Pose2d(line[disp, 1].value(), 0.0),
                    actualPose = currentPosition,
                    actualVelActual = robotVel
                )

                val wheelVels = kinematics.inverse(command)
                val voltage = voltageSensor.voltage

                val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
                val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
                val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
                val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage

                mecanumCommandWriter.write(
                    MecanumCommandMessage(
                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                    )
                )

                leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
                leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
                rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
                rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage
            }.setFinish {
                disp >= line.length
            }
    }
}

fun List<Trajectory>.follow(robot: MecanumChassis): Command {
    return Sequential(this.map { robot.followPathCommand(it) })
}

fun HolonomicController.compute(
    targetPose: Pose2d,
    actualPose: Pose2d,
    actualVelActual: PoseVelocity2d
) = this.compute(
    Pose2dDual.constant(targetPose, 3),
    actualPose,
    actualVelActual
)

