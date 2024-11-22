package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.dependency.feature.SingleFeature
import dev.frozenmilk.dairy.core.util.OpModeFreshLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper

/**
 * Dairy Service (Feature) allowing for the asynchronous usage of actions.
 * Use by adding [SilkRoad.Attach] to your OpMode class.
 * Note: does not work with LinearOpModes
 *
 * @author Iris; adapted for Kotlin use by Zach
 */
object SilkRoad : Feature {
    override var dependency: Dependency<*> = SingleAnnotation(SilkRoad.Attach::class.java)

    private val dash: FtcDashboard by OpModeFreshLazyCell { FtcDashboard.getInstance() }.apply {
        dependency = SingleFeature(this@SilkRoad)
    }
    private val canvas: Canvas by OpModeFreshLazyCell { Canvas() }.apply {
        dependency = SingleFeature(this@SilkRoad)
    }

    private val actions: MutableList<Action> = mutableListOf()


    /**
     * Adds an action to the list of actions running at the end of each loop
     * (AKA runs it asynchronously).
     * @param action action to run
     */
    fun runAsync(action: Action) {
        actions.add(action)
    }

    override fun preUserInitHook(opMode: Wrapper) {
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        val packet = TelemetryPacket()
        packet.fieldOverlay().operations.addAll(canvas.operations)
        val iter: MutableIterator<Action> = actions.iterator()
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

/**
 * Allows for the creation of an Action using init and loop methods and a condition.
 * @param condition stops action when condition is true
 */
abstract class ActionsEx(private val condition: () -> Boolean) : Action {
    /**
     * Initializes the action.
     * This will always run before [loop].
     */
    abstract fun init() : Unit

    /**
     * Contents of the action.
     * This will repeat until [condition] is true.
     */
    abstract fun loop() : Unit

    private var hasInit = false

    final override fun run(p: TelemetryPacket): Boolean {
        if (!hasInit) {
            init()
            hasInit = true
        }
        loop()
        return !condition.invoke();
    }
}

/**
 * Runs all of [actions] in parallel in the order provided.
 * Every call to this will call every action in [actions].
 * When any action is done, all others are force-stopped.
 * @param actions actions to run in parallel
 */
class RaceParallelAction(private vararg val actions: Action) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        var finished = true
        for (action in actions) finished = finished && action.run(p)
        return finished
    }

    override fun preview(fieldOverlay: Canvas) {
        for (action in actions) action.preview(fieldOverlay)
    }
}

/**
 * Runs an action with a configurable timeout in seconds, defaulting to 5 seconds.
 *
 * After the timeout expires, the action will be FORCIBLY stopped.
 * Ensure that this will not leave your robot in a dangerous state (motors still moving, etc.) that is not resolved by another action.
 */
class TimeoutAction(val action: Action, val timeout: Double = 5.0) : Action by action {
    private var startTime = 0.0

    override fun run(t: TelemetryPacket): Boolean {
        if (startTime == 0.0) startTime = now()
        if (now() - startTime > timeout) return false
        return action.run(t)
    }
}

/** Takes two actions and a condition supplier as inputs.
 * Runs the first action if the condition is true and runs the second action if it is false.
 *
 * Written by j5155, released under the MIT License and the BSD-3-Clause license (you may use it under either one)
 *
 * Example usage:
 * In Java:
 * ```java
 * new ConditionalAction(
 *                  new SleepAction(1), // will be run if the conditional is true
 *                  new SleepAction(2), // will be run if the conditional is false
 *                  () -> Math.random() > 0.5); // lambda conditional function, returning either true or false;
 *          // this example picks which one to run randomly
 * ```
 * Or in Kotlin:
 * ```kotlin
 * fun choose(): Action {
 *     return ConditionalAction(
 *         SleepAction(1.0),  // will be run if the conditional is true
 *         SleepAction(2.0) // will be run if the conditional is false
 *     ) { Math.random() > 0.5 }
 *     // lambda conditional function, returning either true or false;
 *     // this example picks which one to run randomly
 * }
 * ```
 */
class ConditionalAction(private val trueAction: Action, private val falseAction: Action, private val condition: () -> Boolean) :
    Action {
    private var chosenAction: Action? = null
    private var initialized = false

    fun init() {
        chosenAction =
            if (condition.invoke()) { // use .get() to check the value of the condition by running the input function
                trueAction // and then save the decision to the chosenAction variable
            } else {
                falseAction
            }
    }

    fun loop(t: TelemetryPacket): Boolean {
        // every loop, pass through the chosen action
        return chosenAction!!.run(t)
    }

    override fun run(p: TelemetryPacket): Boolean {
        if (!initialized) {
            init()
            initialized = true
        }
        return loop(p)
    }

    // ambiguous which one to preview, so preview both
    override fun preview(canvas: Canvas) {
        chosenAction?.preview(canvas)
    }
}
