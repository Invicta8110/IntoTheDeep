package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.control.mtel

@Autonomous
@Mercurial.Attach
class CommandsTest : OpMode() {
    lateinit var command: Command
    val timer = ElapsedTime()

    override fun init() {
        command = Lambda("timer")
            .setInit { timer.reset() }
            .setExecute { mtel.addData("Timer", timer.seconds()) }
            .setFinish { true }
    }

    override fun start() {
        command.schedule()
    }

    override fun loop() {
        mtel.addData("Command Snapshot", Mercurial.activeCommandSnapshot)
        mtel.addData("Scheduled", Mercurial.isScheduled(command))
        mtel.update()
    }
}