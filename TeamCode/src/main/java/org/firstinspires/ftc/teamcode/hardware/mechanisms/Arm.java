package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

@Config
public class Arm {
    //VIA SOME TESTING I'VE DETERMINED THE ARM SHOULD STAY AT PIDFCOEFFICIENTS kP = 0.005, kI = 0, kD = 0.01

    public static int DOWN_POS, UP_POS;
    public static PIDFController.PIDCoefficients coefs = new PIDFController.PIDCoefficients(0.005, 0, 0.01);
    public final Motor motor;

    public Arm(Motor motor) {
        this.motor = motor;
        motor.reverse();
        DOWN_POS = 0;
        UP_POS = 1000;
    }

    public Motor.PIDFAction goUp() {
        Motor.PIDFAction a = motor.new PIDFAction(UP_POS, coefs);
        a.getPidf().setOutputBounds(-0.75, 0.75);
        return a;
    }

    public Motor.PIDFAction goDown() {
        Motor.PIDFAction a = motor.new PIDFAction(DOWN_POS, coefs);
        a.getPidf().setOutputBounds(-0.75, 0.75);
        return a;
    }
}
