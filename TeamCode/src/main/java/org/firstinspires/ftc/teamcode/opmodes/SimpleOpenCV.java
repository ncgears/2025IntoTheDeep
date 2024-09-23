package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.tseSaturationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Test - OpenCV")
@Disabled
public class SimpleOpenCV extends OpMode {
    private tseSaturationProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new tseSaturationProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
    }
}
