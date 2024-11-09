package org.firstinspires.ftc.teamcode.hardware.robots;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.TwoPointServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.hardware.wrappers.Motor;

import java.util.List;

public class ChocolateRaisin {
    private final MecanumChassis chassis;
    private final LinearSlides slides;
    private final Arm arm;
    private final TwoPointServo claw;
    private LynxModule controlHub, expansionHub;

    public ChocolateRaisin(HardwareMap hwMap, Pose2d pose) {
        chassis = new MecanumChassis(hwMap, pose);
        slides = new LinearSlides(new Motor("slides", hwMap));
        arm = new Arm(new Motor("arm", hwMap));
        claw = new TwoPointServo("claw", hwMap);

        List<LynxModule> hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            if (module.isParent() && controlHub == null) {
                controlHub = module;
            } else if (expansionHub == null) {
                expansionHub = module;
            }
        }
    }

    public ChocolateRaisin(HardwareMap hwMap) {
        this(hwMap, new Pose2d(0, 0, 0));
    }

    public MecanumChassis chassis() {
        return chassis;
    }

    public LinearSlides slides() {
        return slides;
    }

    public Arm arm() {
        return arm;
    }

    public TwoPointServo claw() {
        return claw;
    }

    public LynxModule controlHub() {
        return controlHub;
    }

    public LynxModule expansionHub() {
        return expansionHub;
    }

}
