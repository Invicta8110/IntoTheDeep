package org.firstinspires.ftc.teamcode.roadrunner

import android.util.Log
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PinpointView
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.wavedash.GoBildaPinpointDriver
import dev.frozenmilk.wavedash.Localizer

class TwoDeadWheelPinpointImu(
    hardwareMap: HardwareMap,
    val inPerTick: Double,
    initialPose: Pose2d
) : Localizer {
    object PARAMS {
        val parYTicks = 0.0
        val perpXTicks = 0.0
    }

    val pinpoint = hardwareMap[GoBildaPinpointDriver::class.java, "pinpoint"]
    val par = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "backRight"))).apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    val perp = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "frontRight"))).apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    var initialized = false
    var lastParPos = 0
    var lastPerpPos = 0
    var lastHeading = Rotation2d(0.0, 0.0)

    override var pose = initialPose

    override fun update(): PoseVelocity2d {
        pinpoint.update()

        val parPosVel: PositionVelocityPair = par.getPositionAndVelocity()
        val perpPosVel: PositionVelocityPair = perp.getPositionAndVelocity()

        val heading = Rotation2d.exp(pinpoint.heading)
        val headingVel = pinpoint.headingVelocity

        Log.d("Localizer", "parPos: ${parPosVel.position}, perpPos: ${perpPosVel.position}, heading: $heading")

        if (!initialized) {
            initialized = true

            lastParPos = parPosVel.position
            lastPerpPos = perpPosVel.position
            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val parPosDelta: Int = parPosVel.position - lastParPos
        val perpPosDelta: Int = perpPosVel.position - lastPerpPos
        val headingDelta = heading.minus(lastHeading)

        val twist = Twist2dDual<Time>(
            Vector2dDual<Time>(
                DualNum<Time>(
                    doubleArrayOf(
                        parPosDelta - PARAMS.parYTicks * headingDelta,
                        parPosVel.velocity - PARAMS.parYTicks * headingVel,
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        perpPosDelta - PARAMS.perpXTicks * headingDelta,
                        perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                    )
                ).times(inPerTick)
            ),
            DualNum<Time>(
                doubleArrayOf(
                    headingDelta,
                    headingVel,
                )
            )
        )

        lastParPos = parPosVel.position
        lastPerpPos = perpPosVel.position
        lastHeading = heading

        pose = pose + twist.value()
        return twist.velocity().value()
    }

    val view = object : PinpointView {
        override var pose: Pose2d by this@TwoDeadWheelPinpointImu::pose
        override var vel = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)

        override fun update() {
            vel = this@TwoDeadWheelPinpointImu.update()
        }

        override var parDirection: DcMotorSimple.Direction = par.direction
        override var perpDirection: DcMotorSimple.Direction = perp.direction
    }
}