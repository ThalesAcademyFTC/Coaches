package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

public class TestBot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Hashtable<String, Telemetry.Item> telemetryData;
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftRearMotor;
    DcMotorEx rightRearMotor;
    IMU imu;
    ColorRangeSensor colorSensor;
    Rev2mDistanceSensor distanceSensor;

    // member variables used for robot vision via the webcam
    WebcamName webcamName;
    VisionPortal visionPortal = null;
    AprilTagProcessor aprilTagProcessor = null;
    ColorBlobLocatorProcessor colorBlobLocatorProcessor = null;
    PredominantColorProcessor predominantColorProcessor = null;

    TestBot(HardwareMap hMap, Telemetry tm) {
        hardwareMap = hMap;
        telemetry = tm;
        telemetry.setAutoClear(false);
        telemetryData = new Hashtable<>();

        // the webcam might not be attached
        webcamName = hMap.tryGet(WebcamName.class, "Webcam 1");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        leftFrontMotor  = hMap.get(DcMotorEx.class,"motorFrontLeft");
        rightFrontMotor = hMap.get(DcMotorEx.class,"motorFrontRight");
        leftRearMotor   = hMap.get(DcMotorEx.class,"motorBackLeft");
        rightRearMotor  = hMap.get(DcMotorEx.class,"motorBackRight");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hMap.get(RevColorSensorV3.class, "colorSensor");
        distanceSensor = hMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    void initializeAprilTagDetection() {
        if(webcamName == null) {
            return;
        }
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    void initializeColorBlobDetection(ColorRange colorToFind) {
        if(webcamName == null) {
            return;
        }
        colorBlobLocatorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorToFind)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000);
        colorBlobLocatorProcessor.addFilter(areaFilter);
        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorBlobLocatorProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(webcamName)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetryData.put("blobsFound", telemetry.addData("blobs detected", 0));
    }

    public void locateColorBlobs() {
        if (colorBlobLocatorProcessor != null) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorBlobLocatorProcessor.getBlobs();

            update("blobsFound", blobs.size());
        }
    }

    public void initializePredominantColorDetection() {
        PredominantColorProcessor.Swatch[] swatches = new PredominantColorProcessor.Swatch[2];
        swatches[0] = PredominantColorProcessor.Swatch.RED;
        swatches[1] = PredominantColorProcessor.Swatch.ORANGE;
        predominantColorProcessor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.25, 0.25, 0.25, -0.25))
                .setSwatches(swatches)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(predominantColorProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(webcamName)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }
    public PredominantColorProcessor.Swatch getPredominateColor() {
        if (predominantColorProcessor != null) {
            return predominantColorProcessor.getAnalysis().closestSwatch;
        }
        return null;
    }
    void addSensorTelemetry() {
        telemetry.addData("imu heading    ", "% .2f degrees", this::getIMUHeadingDegrees);
        telemetry.addData("object distance", "%.2f cm", this::getDistance);
    }
    void addDriveMotorTelemetry(boolean usingEncoders) {
        Telemetry.Line line;
        String prefix = "leftFront";

        line = telemetry.addLine("Left Front   ");
        telemetryData.put(prefix + "Port", line.addData("Port", () -> leftFrontMotor.getPortNumber()));
        telemetryData.put(prefix + "Position", line.addData("Pos", "% 6d", () -> leftFrontMotor.getCurrentPosition()));
        if (usingEncoders) {
            telemetryData.put(prefix + "Target", line.addData("Tar", "% 6d", () -> leftFrontMotor.getTargetPosition()));
        } else {
            telemetryData.put(prefix + "Power", line.addData("Pow", "% .2f", () -> leftFrontMotor.getPower()));
        }
        line = telemetry.addLine("Right Front ");
        prefix = "rightFront";
        telemetryData.put(prefix + "Port", line.addData("Port", () -> rightFrontMotor.getPortNumber()));
        telemetryData.put(prefix + "Position", line.addData("Pos", "% 6d", () -> rightFrontMotor.getCurrentPosition()));
        if (usingEncoders) {
            telemetryData.put(prefix + "Target", line.addData("Tar", "% 6d", () -> rightFrontMotor.getTargetPosition()));
        } else {
            telemetryData.put(prefix + "Power", line.addData("Pow", "% .2f", () -> rightFrontMotor.getPower()));
        }
        line = telemetry.addLine("Left Rear     ");
        prefix = "leftRear";
        telemetryData.put(prefix + "Port", line.addData("Port", () -> leftRearMotor.getPortNumber()));
        telemetryData.put(prefix + "Position", line.addData("Pos", "% 6d", () -> leftRearMotor.getCurrentPosition()));
        if (usingEncoders) {
            telemetryData.put(prefix + "Target", line.addData("Tar", "% 6d", () -> leftRearMotor.getTargetPosition()));
        } else {
            telemetryData.put(prefix + "Power", line.addData("Pow", "% .2f", () -> leftRearMotor.getPower()));
        }
        line = telemetry.addLine("Right Rear   ");
        prefix = "rightRear";
        telemetryData.put(prefix + "Port", line.addData("Port", () -> rightRearMotor.getPortNumber()));
        telemetryData.put(prefix + "Position", line.addData("Pos", "% 6d", () -> rightRearMotor.getCurrentPosition()));
        if (usingEncoders) {
            telemetryData.put(prefix + "Target", line.addData("Tar", "% 6d", () -> rightRearMotor.getTargetPosition()));
        } else {
            telemetryData.put(prefix + "Power", line.addData("Pow", "% .2f", () -> rightRearMotor.getPower()));
        }
    }

    // update given telemetry item's value
    void update(String caption, double value) {
        Telemetry.Item item = telemetryData.get(caption);
        if (item != null) {
            item.setValue(value);
        }
    }
    public void showGamePad(Gamepad gamepad) {
        update("leftStickX", gamepad.left_stick_x);
        update("leftStickY", gamepad.left_stick_y);
    }
    /*
    public void addTelemetry(String caption, String format, Func<> function) {
        Telemetry.Item item = telemetry.addData(caption, format, function);
        telemetryData.put(caption, item);
    } */
    double getIMUHeading() {
        YawPitchRollAngles yawPitchRoll = imu.getRobotYawPitchRollAngles();
        return yawPitchRoll.getYaw(AngleUnit.RADIANS);
    }
    double getIMUHeadingDegrees() {
        YawPitchRollAngles yawPitchRoll = imu.getRobotYawPitchRollAngles();
        return yawPitchRoll.getYaw(AngleUnit.DEGREES);
    }
    double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    public void drive(double forward, double right, double rotateCW) {
        double lf = right + forward + rotateCW;
        double rf = -right + forward - rotateCW;
        double lr = -right + forward + rotateCW;
        double rr = right + forward - rotateCW;

        double maxOr1 = Math.max(Math.abs(lf),
                Math.max(Math.abs(rf),
                        Math.max(Math.abs(lr),
                                Math.max(Math.abs(rr), 1.0))));
        // scale all 4 power values evenly to get them in the -1..1 range
        // of the setPower() function, but only if the absolute max power value
        // is greater than 1
        lf /= maxOr1;
        rf /= maxOr1;
        lr /= maxOr1;
        rr /= maxOr1;

        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftRearMotor.setPower(lr);
        rightRearMotor.setPower(rr);
    }

    public void driveAbsolute(double up, double right, double rotateCW) {
        // figure out direction from up and right
        double v1 = Math.sqrt((up*up) + (right*right));
        double theta = Math.atan2(right, up);
        double beta = getIMUHeading();
        double alpha = (Math.PI / 2) - beta - theta;
        double x2 = Math.cos(alpha) * v1;
        double y2 = Math.sin(alpha) * v1;
        drive(y2, x2, rotateCW);
    }


}
