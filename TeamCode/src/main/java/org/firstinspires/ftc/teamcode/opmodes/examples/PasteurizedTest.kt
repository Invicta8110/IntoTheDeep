package org.firstinspires.ftc.teamcode.opmodes.examples

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class PasteurizedTest : OpMode() {
    val drive: MecanumDrive by lazy { MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }
    val slides: LinearSlides by lazy { LinearSlides(hardwareMap) }
    val running = mutableListOf<Action>()

    val gamepad = Pasteurized.gamepad1

    override fun init() {
        drive
        slides
    }

    override fun loop() {
        val new = mutableListOf<Action>()

        val packet = TelemetryPacket()

        if (gamepad.a.onTrue) {
            running.add(slides.goUp)
        }

        for (a in running) {
            a.preview(packet.fieldOverlay())
            if (a.run(packet)) {
                new.add(a)
            }
        }
    }
}