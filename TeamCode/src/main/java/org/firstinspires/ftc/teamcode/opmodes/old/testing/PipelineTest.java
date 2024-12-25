package org.firstinspires.ftc.teamcode.opmodes.old.testing;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.control.vision.SampleDetectionPipelinePNP;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleDetectionPipelinePNP pipeline = new SampleDetectionPipelinePNP();
        VisionProcessor processor = processorFromPipeline(pipeline);

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(CameraName.class, "Webcam 1"),
                processor
        );
    }

    public static VisionProcessor processorFromPipeline(OpenCvPipeline pipeline) {
        return new VisionProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {

            }

            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                return pipeline.processFrame(frame);
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            }
        };
    }
}
