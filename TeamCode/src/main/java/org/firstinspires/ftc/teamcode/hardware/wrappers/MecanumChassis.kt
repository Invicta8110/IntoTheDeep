package org.firstinspires.ftc.teamcode.hardware.wrappers

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class MecanumChassis(val dt: MecanumDrive) {

    @JvmOverloads
    constructor(hwMap: HardwareMap, pose: Pose2d = Pose2d(0.0, 0.0, 0.0)) :
            this(MecanumDrive(hwMap, pose))

    fun moveToPoint(x: Double, y: Double) : Action {
        return dt.actionBuilder(dt.pose)
            .splineTo(Vector2d(x, y), dt.pose.heading)
            .build()
    }

}