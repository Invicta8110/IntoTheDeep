package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.ActionOpMode;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

@Disabled
@Autonomous
public class SlideTest extends OpMode {
    Motor slides;
    LinearSlides.SlidePosition target = LinearSlides.SlidePosition.DOWN;
    final int DOWN_POS = 0, UP_POS = 1000;
    //PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> gp1;

    @Override
    public void init() {
        slides = new Motor("slides", hardwareMap);
        //gp1 = Pasteurized.gamepad1();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            target = LinearSlides.SlidePosition.UP;
            slides.runToPosition(UP_POS, .6);
        } else if (gamepad1.dpad_down) {
            target = LinearSlides.SlidePosition.DOWN;
            slides.runToPosition(DOWN_POS, .6);
        }

        telemetry.addData("Status", target);
    }
}
