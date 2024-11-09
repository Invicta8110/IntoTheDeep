package org.firstinspires.ftc.teamcode.opmodes.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class ColorDetector extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal camera = VisionPortal.easyCreateWithDefaults(hardwareMap.get(CameraName.class, "Webcam 1"),
                colorSensor);

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("Color", colorSensor.getAnalysis().closestSwatch);
        }
    }
}
