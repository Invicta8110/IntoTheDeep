package org.firstinspires.ftc.teamcode.control

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.sinister.opmode.SinisterOpModes
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta


fun SinisterOpModes.register(meta: OpModeMeta, opMode: OpMode) {
    register(meta, opMode::class.java)
}