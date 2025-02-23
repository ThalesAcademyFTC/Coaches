package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Sensor Test")
public class TestBotSensorTest extends OpMode {
    TestBot testBot;

    public void init() {
        testBot = new TestBot(hardwareMap, telemetry);
        testBot.addDriveMotorTelemetry(false);
        testBot.addSensorTelemetry();
    }

    public void loop() {
        testBot.showGamePad(gamepad1);

        //testBot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        testBot.driveAbsolute(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
    }
}
