package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

class MotorServoDocsKT : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        //you have to do all of your initialization inside the runOpMode method (or the superclass methods of the iterative OpMode class)

        //in kotlin it has to be done inside the method of choice bc Kotlin doesnt like null objects
        val motorEx = hardwareMap.get(
            DcMotor::class.java,
            "motor"
        ) //this tells the control hub that you are looking for whichever motor is labeled as "motor" in the config file
        val servoEx = hardwareMap.get(Servo::class.java, "servo")
        val crServoEx = hardwareMap.get(CRServo::class.java, "crservo")
        waitForStart()

        //MOTORS
        //Motors are pretty easy: they just kinda go (mostly)
        motorEx.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER //for basic use we do not need the encoder; kotlin uses access and modification syntax to use get and set methods
        motorEx.power = 1.0 //setPower is on a scale of -1 to 1
        motorEx.power = 0.0

        //we could also start including the encoders
        motorEx.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER //this sets the position to 0
        motorEx.mode = DcMotor.RunMode.RUN_TO_POSITION //and now it will do whatever it can to run to the desired position once I set it
        motorEx.targetPosition = 538 * 4 //according to goBILDA, there are about 538 motor ticks per revolution, so to get it to rotate 4 times we multiply it by 4
        motorEx.power = 0.5

        //ok motor time is up
        motorEx.power = 0.0

        //SERVOS
        //Servos are designed to travel to precise positions expressed as a percent of their total range
        //goBILDA servos by default have a range of 300 degrees (but this can again be changed with a servo programmer)
        servoEx.position = .5 //this will make the servo travel 150 of those 300 degrees
        servoEx.position = 0.0 //and now it will go back to the default position

        //CRServos are again just servos but they act like motors
        crServoEx.power = 1.0 //it will now run at its full power
        sleep(1000)
        crServoEx.power = 0.0 //and now one second later it stops
    }
}
