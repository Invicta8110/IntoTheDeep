package org.firstinspires.ftc.teamcode.control.services

import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.lazy.Yielding
import dev.frozenmilk.dairy.core.wrapper.Wrapper

import org.firstinspires.ftc.teamcode.control.PIDFController
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor

class PIDFService(val motor: Motor, val controller: PIDFController) : Feature {
	// first, we need to set up the dependency
	// Yielding just says "this isn't too important, always attach me, but run me after more important things"
	// Yielding is reusable!
	override var dependency: Dependency<*> = Yielding

	init {
		// regardless of constructor used, call register when the class is instantiated
		register()
	}

	private fun update() {
		val output = controller.update(motor.position.toDouble())

		// don't update motor power if the controller isn't enabled
		if (enabled) {
			motor.power = output
		} else {
			return
		}
	}

	// users should be able to change the target
	var target: Int by controller::targetPosition

	// users should be able to enable / disable the controller
	var enabled: Boolean = true


	// update controller after loop
	override fun postUserLoopHook(opMode: Wrapper) {
		update()
	}

	// in cleanup we deregister, which prevents this from sticking around for another OpMode,
	// unless the user calls register again
	override fun cleanup(opMode: Wrapper) {
		deregister()
	}
}