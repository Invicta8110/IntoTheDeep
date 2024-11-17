package org.firstinspires.ftc.teamcode.control.services

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import java.lang.annotation.Inherited

class SilkRoad private constructor() : Feature {
    override var dependency: Dependency<*> = SingleAnnotation(
        Attach::class.java
    )

    override fun postUserInitHook(opMode: Wrapper) {
        dash = FtcDashboard.getInstance()
        canvas = Canvas()
        run = true
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        if (run && !Thread.currentThread().isInterrupted) {
            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas!!.operations)

            run = actions!!.run(packet)

            dash!!.sendTelemetryPacket(packet)
        }
    }

    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @Inherited
    annotation class Attach

    companion object {
        private var dash: FtcDashboard? = null
        private var canvas: Canvas? = null
        private var actions: Action? = null
        private var run = false
        val INSTANCE: SilkRoad = SilkRoad()

        fun runAsync(actions: Action?) {
            Companion.actions = actions
        }
    }
}