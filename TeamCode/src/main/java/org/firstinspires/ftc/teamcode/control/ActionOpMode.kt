package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
import java.util.function.BooleanSupplier

/**
 * Makes a
 */
abstract class ActionOpMode : OpMode() {
    private var running = mutableListOf<Action>()
    val packet : TelemetryPacket by lazy { TelemetryPacket() }

    fun add(action: Action) {
        running.add(action)
    }

    override fun loop() {
        val new = mutableListOf<Action>()

        val packet = TelemetryPacket()

        for (a in running) {
            a.preview(packet.fieldOverlay())
            if (a.run(packet)) {
                new.add(a)
            }
        }

        running = new
    }

    fun runActionOnPress(action: Action, button: EnhancedBooleanSupplier) {
        if (button.onTrue) {
            add(action)
        }
    }

}