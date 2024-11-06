package org.firstinspires.ftc.teamcode.control;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class RedBlueDetectionPipelineNoPNP implements VisionProcessor {
    /*
     * Threshold values
     */
    static final int YELLOW_MASK_THRESHOLD = 57;
    static final int BLUE_MASK_THRESHOLD = 150;
    static final int RED_MASK_THRESHOLD = 198;
    /*
     * Colors
     */
    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);
    static final int CONTOUR_LINE_THICKNESS = 2;
    /*
     * Our working image buffers
     */
    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();
    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();
    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();
    Stage[] stages = Stage.values();
    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }

    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Mat processFrame(Mat frame, long milliseconds) {
        // We'll be updating this with new data below
        internalStoneList.clear();

        /*
         * Run the image processing
         */
        findContours(frame);

        clientStoneList = new ArrayList<>(internalStoneList);

        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum]) {
            case YCrCb: {
                return ycrcbMat;
            }

            case FINAL: {
                return frame;
            }

            case MASKS: {
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);
                return masks;
            }

            case MASKS_NR: {
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);
                return masksNR;
            }

            case CONTOURS: {
                return contoursOnPlainImageMat;
            }
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public ArrayList<AnalyzedStone> getDetectedStones() {
        return clientStoneList;
    }

    void findContours(Mat input) {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel for blue detection
        Core.extractChannel(ycrcbMat, cbMat, 2); // Cb channel index is 2

        // Extract the Cr channel for red detection
        Core.extractChannel(ycrcbMat, crMat, 1); // Cr channel index is 1

        // Threshold the Cb channel to form a mask for blue
        Imgproc.threshold(cbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        // Threshold the Cr channel to form a mask for red
        Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        // Threshold the Cb channel to form a mask for yellow
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the masks
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Now analyze the contours
        for (MatOfPoint contour : blueContoursList) {
            analyzeContour(contour, input, "Blue");
        }

        for (MatOfPoint contour : redContoursList) {
            analyzeContour(contour, input, "Red");
        }

        for (MatOfPoint contour : yellowContoursList) {
            analyzeContour(contour, input, "Yellow");
        }
    }

    void morphMask(Mat input, Mat output) {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color) {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRotatedRect(rotatedRectFitToContour, input, color);

        // The angle OpenCV gives us can be ambiguous, so look at the shape of
        // the rectangle to fix that.
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
            rotRectAngle += 90;
        }

        // Compute the angle and store it
        double angle = -(rotRectAngle - 180);
        drawTagText(rotatedRectFitToContour, (int) Math.round(angle) + " deg", input, color);

        AnalyzedStone analyzedStone = new AnalyzedStone();
        analyzedStone.angle = rotRectAngle;
        analyzedStone.color = color;
        internalStoneList.add(analyzedStone);
    }

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS
    }

    static class AnalyzedStone {
        double angle;
        String color;
    }
}