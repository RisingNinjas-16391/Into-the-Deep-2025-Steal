package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    private FtcDashboard dash = FtcDashboard.getInstance();

    public ElapsedTime timer = null;
    public ElapsedTime buttonTimer = null;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;
        startingPose = new Pose2d(0, 0, 0);

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when tele-op starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            timer = new ElapsedTime();
            buttonTimer = new ElapsedTime();
            robot.initHasMovement();
            // Color Sensor to detect sample in intake
            robot.colorSensor.enableLed(true);
        }
        // Endgame/hang rumble after 105 seconds to notify robot.driver to hang
        else if ((timer.seconds() > 105) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        TelemetryPacket packet = new TelemetryPacket();

        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        if (distance < 1.5) {
            if (red >= green && red >= blue) {
                currentSample = SampleDetected.RED;
            } else if (green >= red && green >= blue) {
                currentSample = SampleDetected.YELLOW;
            } else {
                currentSample = SampleDetected.BLUE;
            }
        }
        else {
            currentSample = SampleDetected.NONE;
        }

        // Gamepad Lights (untested)
        if (currentSample.equals(RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (currentSample.equals(BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (currentSample.equals(YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // OTOS Field Centric robot.Drive Code
        robot.drive.updatePoseEstimate();
        robot.drive.setFieldCentricDrivePowers(
            new PoseVelocity2d(
                new Vector2d((driver.getLeftY()), (driver.getLeftX())),
                driver.getRightX()),
                driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                robot.drive.pose.heading.toDouble()
        );

        // Reset IMU for field centric
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            robot.drive.pose = new Pose2d(0, 0,0);
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        // DO NOT REMOVE! Needed for telemetry
        telemetry.update();
        dash.sendTelemetryPacket(packet);

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}