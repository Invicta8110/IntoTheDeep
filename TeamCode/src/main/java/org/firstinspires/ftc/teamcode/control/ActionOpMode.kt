package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import java.util.function.BooleanSupplier

/**
 * An OpMode that allows for the easy creation and execution of Actions.
 * Actions are run in ActionOpMode's loop method, which must be called at the end of the loop method of the extending class.
 */
abstract class ActionOpMode : OpMode() {
    private var running = mutableListOf<Action>()

    /**
     * gamepad1 as an SDK Gamepad from Pasteurized
     */
    @get:JvmName("gp1")
    val gp1 : SDKGamepad by lazy { SDKGamepad(gamepad1) }

    /**
     * gamepad2 as an SDK Gamepad from Pasteurized
     */
    @get:JvmName("gp2")
    val gp2 : SDKGamepad by lazy { SDKGamepad(gamepad2) }

    /**
     * Forces both SDKGamepads to initialize (since they are evaluated lazily).
     * Should be called at the beginning of the init method of the extending class.
     */
    override fun init() {
        gp1
        gp2
    }

    /**
     * Adds an Action to the list of Actions to be run at the end of the loop.
     * @param action The Action to add
     */
    fun add(action: Action) {
        running.add(action)
    }

    /**
     * Runs all Actions in the list of running Actions.
     * Should be called at the end of the loop method of the extending class.
     */
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

    /**
     * Adds an Action to the list of running Actions if the given button is pressed.
     * @param action The Action to add
     * @param button The button to check
     */
    fun addActionOnPress(action: Action, button: EnhancedBooleanSupplier) {
        if (button.onTrue) {
            add(action)
        }
    }
}