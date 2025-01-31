package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SleepAction
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.dependency.feature.SingleFeature
import dev.frozenmilk.dairy.core.util.OpModeFreshLazyCell
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import page.j5155.expressway.core.actions.RaceParallelAction

/**
 * Dairy Service (Feature) allowing for the asynchronous usage of runningActions.
 * Use by adding [SilkRoad.Attach] to your OpMode class.
 * Note: does not work with LinearOpModes
 *
 * @author Iris; adapted for Kotlin use by Zach
 */
object SilkRoad : Feature {
    override var dependency: Dependency<*> = SingleAnnotation(Attach::class.java)

    private val dash: FtcDashboard by OpModeFreshLazyCell { FtcDashboard.getInstance() }.apply {
        dependency = SingleFeature(this@SilkRoad)
    }
    private val canvas: Canvas by OpModeFreshLazyCell { Canvas() }.apply {
        dependency = SingleFeature(this@SilkRoad)
    }

    private val runningActions: MutableList<Action> = mutableListOf()
    val publicActions: List<Action>
        get() = runningActions


    /**
     * Adds an action to the list of runningActions running at the end of each loop
     * (AKA runs it asynchronously).
     * @param action action to run
     */
    @JvmStatic
    fun runAsync(action: Action) {
        runningActions.add(action)
    }

    override fun preUserInitHook(opMode: Wrapper) {
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        val packet = TelemetryPacket()
        packet.fieldOverlay().operations.addAll(canvas.operations)
        val iter: MutableIterator<Action> = runningActions.iterator()
        while (iter.hasNext() && !Thread.currentThread().isInterrupted) {
            val action: Action = iter.next()
            if (!action.run(packet)) iter.remove()
        }
        dash.sendTelemetryPacket(packet)
    }

    /**
     * Attaches the SilkRoad service to an OpMode.
     */
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    annotation class Attach
}

fun Action.timeout(timeout: Double) = RaceParallelAction(SleepAction(timeout), this)

fun EnhancedBooleanSupplier.onPress(action: Action) {
    SilkRoad.runAsync(action)
}