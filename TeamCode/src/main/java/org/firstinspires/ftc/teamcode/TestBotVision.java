package org.firstinspires.ftc.teamcode;

import android.icu.number.Scale;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

@Autonomous(name="Color Blob Detection")
public class TestBotVision extends LinearOpMode {

    public void runOpMode() {
        TestBot testBot = new TestBot(hardwareMap, telemetry);

        testBot.addDriveMotorTelemetry(true);
        testBot.addSensorTelemetry();

        // do any initialization work here
        ColorRange color = new ColorRange(ColorSpace.RGB, new Scalar(50,0,0), new Scalar(255,40,40));
        testBot.initializeColorBlobDetection(color);

        while(opModeInInit()) {
            testBot.locateColorBlobs();
            telemetry.update();
            sleep(50);
        }

        while(opModeIsActive()) {
            // do something with blob detection info
            sleep(50);
        }
    }
}
