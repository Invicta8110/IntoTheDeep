package org.firstinspires.ftc.teamcode.opmodes.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class SampleLocator extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

//        ColorBlobLocatorProcessor colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
//                .setDrawContours(true)                        // Show contours on the Stream Preview
//                .setBlurSize(5)                               // Smooth the transitions between different colors in image
//                .build();
//
//        ColorBlobLocatorProcessor colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
//                .setDrawContours(true)                        // Show contours on the Stream Preview
//                .setBlurSize(5)                               // Smooth the transitions between different colors in image
//                .build();

        VisionPortal camera = VisionPortal.easyCreateWithDefaults(hardwareMap.get(CameraName.class, "Webcam 1"),
                colorLocatorBlue);

        while (opModeIsActive() || opModeInInit()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocatorBlue.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(50, 10000, blobs);

            for (ColorBlobLocatorProcessor.Blob b: blobs) {
                telemetry.addData("Blob", b.getContourArea());
            }

            telemetry.update();
        }
    }
}
