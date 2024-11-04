package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.hardware.TestTeleOpRobot;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    private TestTeleOpRobot robot;

    public void runOpMode() {
        robot = new TestTeleOpRobot(hardwareMap,gamepad1);
        waitForStart();
        while(opModeIsActive()) {
            robot.slides();
            robot.arm();
            robot.drive();

            telemetry.addData("Control Hub Voltage",
                    robot.getControlHub().getInputVoltage(VoltageUnit.VOLTS)
            );
            telemetry.addData("Expansion Hub Voltage",
                    robot.getExpansionHub().getCurrent(CurrentUnit.AMPS)
            );
        }
    }

}
