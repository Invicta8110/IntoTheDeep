package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.control.SampleDetectionPipelinePNP
import org.firstinspires.ftc.vision.VisionPortal

@Autonomous
class SampleScannerOld : OpMode() {
    private lateinit var processor: SampleDetectionPipelinePNP
    private lateinit var camera: VisionPortal

    override fun init() {
        processor = SampleDetectionPipelinePNP()

        camera = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            processor,
        )
    }

    override fun loop() {
        val stones = processor.detectedStones
        for (s in stones) { //for (AnalyzedStone s: stones)
            telemetry.addData("Stone", "Color: ${s.color}, Rvec: ${s.rvec}, Tvec: ${s.tvec}")
        }
        telemetry.update();
    }
}