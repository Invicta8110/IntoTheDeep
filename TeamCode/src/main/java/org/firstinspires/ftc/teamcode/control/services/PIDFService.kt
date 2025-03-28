package org.firstinspires.ftc.teamcode.control.services

import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.lazy.Yielding
import dev.frozenmilk.dairy.core.wrapper.Wrapper

import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor
import page.j5155.expressway.ftc.motion.PIDFController
import kotlin.math.absoluteValue

class PIDFService(val controller: PIDFController, vararg val motors: Motor) : Feature {
	constructor(controller: PIDFController, target: Int, vararg motors: Motor) : this(controller, *motors) {
		this.target = target
	}

	// first, we need to set up the dependency
	// Yielding just says "this isn't too important, always attach me, but run me after more important things"
	// Yielding is reusable!
	override var dependency: Dependency<*> = Yielding
	var lastOutput = 0.0
		private set

	init {
		// regardless of constructor used, call register when the class is instantiated
		register()
	}

	// users should be able to change the target
	var target by controller::targetPosition

	// users should be able to enable / disable the controller
	var enabled: Boolean = true

	// update controller after loop
	override fun postUserLoopHook(opMode: Wrapper) {
		lastOutput = controller.update(
			measuredPosition=motors.first().currentPosition.toDouble(),
			measuredVelocity=motors.first().velocity.toDouble()
		)

		if (enabled) {
			motors.forEach { it.power = lastOutput }
		}
	}

	fun atTarget(tolerance: Int = 20) = controller.lastError.absoluteValue < tolerance

	// in cleanup we deregister, which prevents this from sticking around for another OpMode,
	// unless the user calls register again
	override fun cleanup(opMode: Wrapper) {
		deregister()
	}
}

fun controllerFromPID(kP: Double, kI: Double, kV: Double) = PIDFController(PIDFController.PIDCoefficients(kP, kI, kV))

fun <E> Array<E>.averageOf(func: (E) -> Double) = this.map(func).average()

fun PIDFController.update(
	measuredPosition: Double,
	measuredVelocity: Double
) = this.update(
	timestamp = System.nanoTime(),
	measuredPosition = measuredPosition,
	measuredVelocity = measuredVelocity
)

