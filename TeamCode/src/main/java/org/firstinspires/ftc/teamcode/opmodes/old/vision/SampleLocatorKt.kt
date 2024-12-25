package org.firstinspires.ftc.teamcode.opmodes.old.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc
import java.util.stream.Collectors

class SampleLocatorKt : LinearOpMode() {
    override fun runOpMode() {
        val colorLocatorBlue = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE) // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // exclude blobs inside blobs
            .setRoi(
                ImageRegion.asUnityCenterCoordinates(
                    -0.5,
                    0.5,
                    0.5,
                    -0.5
                )
            ) // search central 1/4 of camera view
            .setDrawContours(true) // Show contours on the Stream Preview
            .setBlurSize(5) // Smooth the transitions between different colors in image
            .build()

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
        val camera = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(
                CameraName::class.java, "Webcam 1"
            ),
            colorLocatorBlue
        )

        while (opModeIsActive() || opModeInInit()) {
            val blobs = colorLocatorBlue.blobs

            val largestBlob = blobs.maxBy { it.contourArea }

            ColorBlobLocatorProcessor.Util.filterByArea(50.0, 10000.0, blobs)

            for (b in blobs) {
                telemetry.addData("Blob Area", b.contourArea)
                telemetry.addData("Blob Center", b.boxFit.center)
                telemetry.addData("Blob Dimensions", b.boxFit.size)
            }

            telemetry.addData("Largest Blob Area", largestBlob)
            telemetry.addData("Largest Blob Center", largestBlob.boxFit.center)
            telemetry.addData("Largest Blob Dimensions", largestBlob.boxFit.size)

            telemetry.update()
        }
    }
}
