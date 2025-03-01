package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.VelConstraint
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import org.firstinspires.ftc.teamcode.control.MecanumParams.inPerTick
import org.firstinspires.ftc.teamcode.control.MecanumParams.lateralInPerTick
import org.firstinspires.ftc.teamcode.control.MecanumParams.maxAngAccel
import org.firstinspires.ftc.teamcode.control.MecanumParams.maxAngVel
import org.firstinspires.ftc.teamcode.control.MecanumParams.maxProfileAccel
import org.firstinspires.ftc.teamcode.control.MecanumParams.maxWheelVel
import org.firstinspires.ftc.teamcode.control.MecanumParams.minProfileAccel
import org.firstinspires.ftc.teamcode.control.MecanumParams.trackWidthTicks
import kotlin.reflect.full.memberProperties

object MecanumParams {
    // IMU orientation
    // TODO: fill in these values based on
    //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
    @JvmStatic
    var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
    @JvmStatic
    var usbFacingDirection: UsbFacingDirection = UsbFacingDirection.FORWARD

    // drive model parameters
    @JvmStatic
    var inPerTick: Double = 117.0 / (46216 - 6238) //5206 ticks
        // / (25.4 * goBILDA_SWINGARM_POD) //ticks-per-mm for the goBILDA Swingarm Pod //ticks-per-mm for the goBILDA Swingarm Pod // If you're using OTOS/Pinpoint leave this at 1 (all values will be in inches, 1 tick = 1 inch)
    @JvmStatic
    var lateralInPerTick: Double =
        0.916176790984675 // Tune this with LateralRampLogger (even if you use OTOS/Pinpoint)
    @JvmStatic
    var trackWidthTicks: Double = 10.5

    // feedforward parameters (in tick units)
    @JvmStatic
    var kS: Double = 3.63265493223649
    @JvmStatic
    var kV: Double = 3.269727019879922
    @JvmStatic
    var kA: Double = 0.0

    // path profile parameters (in inches)
    @JvmStatic
    var maxWheelVel: Double = 50.0
    @JvmStatic
    var minProfileAccel: Double = -30.0
    @JvmStatic
    var maxProfileAccel: Double = 50.0

    // turn profile parameters (in radians)
    @JvmStatic
    var maxAngVel: Double = Math.PI // shared with path
    @JvmStatic
    var maxAngAccel: Double = Math.PI

    // path controller gains
    @JvmStatic
    var axialGain: Double = 0.0
    @JvmStatic
    var lateralGain: Double = 0.0
    @JvmStatic
    var headingGain: Double = 0.0 // shared with turn

    @JvmStatic
    var axialVelGain: Double = 0.0
    @JvmStatic
    var lateralVelGain: Double = 0.0
    @JvmStatic
    var headingVelGain: Double = 0.0 // shared with turn

    @JvmStatic
    val toString = this::class.memberProperties.toString()
}

object MecanumStatic {
    @JvmStatic
    val feedforward = MotorFeedforward(
        MecanumParams.kS,
        MecanumParams.kV / inPerTick,
        MecanumParams.kA / inPerTick
    )

    @JvmStatic
    val controller = HolonomicController(
        MecanumParams.axialGain, MecanumParams.lateralGain, MecanumParams.headingGain,
        MecanumParams.axialVelGain, MecanumParams.lateralVelGain, MecanumParams.headingVelGain
    )

    @JvmStatic
    val kinematics = MecanumKinematics(
        inPerTick * trackWidthTicks, inPerTick / lateralInPerTick
    )

    @JvmStatic
    val defaultTurnConstraints = TurnConstraints(
        maxAngVel, -maxAngAccel, maxAngAccel
    )

    @JvmStatic
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(maxWheelVel),
            AngularVelConstraint(maxAngVel)
        )
    )

    @JvmStatic
    val defaultAccelConstraint: AccelConstraint = ProfileAccelConstraint(
        minProfileAccel, maxProfileAccel
    )
}