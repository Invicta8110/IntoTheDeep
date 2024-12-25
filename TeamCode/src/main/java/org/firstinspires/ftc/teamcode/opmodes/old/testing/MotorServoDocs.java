package org.firstinspires.ftc.teamcode.opmodes.old.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorServoDocs extends LinearOpMode {
    DcMotor motorEx;
    Servo servoEx; //physically, a Servo and a CR Servo (Continuous Rotation Servo) are the same; you use a Servo Programmer to change them from one mode to the other
    CRServo crServoEx; //CRServos are programmed basically the same way as motors, while Servos are not


    @Override
    public void runOpMode() throws InterruptedException {
        //you have to do all of your initialization inside the runOpMode method (or the superclass methods of the iterative OpMode class)
        motorEx = hardwareMap.get(DcMotor.class, "motor"); //this tells the control hub that you are looking for whichever motor is labeled as "motor" in the config file
        servoEx = hardwareMap.get(Servo.class, "servo");
        crServoEx = hardwareMap.get(CRServo.class, "crservo");

        waitForStart();

        //MOTORS
        //Motors are pretty easy: they just kinda go (mostly)
        motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //for basic use we do not need the encoder
        motorEx.setPower(1); //setPower is on a scale of -1 to 1
        motorEx.setPower(0);

        //we could also start including the encoders
        motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //this sets the position to 0
        motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION); //and now it will do whatever it can to run to the desired position once I set it
        motorEx.setTargetPosition(538 * 4); //according to goBILDA, there are about 538 motor ticks per revolution, so to get it to rotate 4 times we multiply it by 4
        motorEx.setPower(0.5);

        //ok motor time is up
        motorEx.setPower(0);

        //SERVOS
        //Servos are designed to travel to precise positions expressed as a percent of their total range
        //goBILDA servos by default have a range of 300 degrees (but this can again be changed with a servo programmer)
        servoEx.setPosition(.5); //this will make the servo travel 150 of those 300 degrees
        servoEx.setPosition(0); //and now it will go back to the default position

        //CRServos are again just servos but they act like motors
        crServoEx.setPower(1); //it will now run at its full power
        sleep(1000);
        crServoEx.setPower(0); //and now one second later it stops
    }
}
