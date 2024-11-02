package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected.*;
import static org.firstinspires.ftc.teamcode.hardware.System.checkButton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    public SparkFunOTOSDrive drive;

    public ElapsedTime timer = null;
    public ElapsedTime buttonTimer = null;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;
    String buttons = "";

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;
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
        if (timer == null) {
            timer = new ElapsedTime();
            buttonTimer = new ElapsedTime();
        }
        // Endgame/hang rumble after 105 seconds to notify driver to hang
        else if ((timer.seconds() > 105) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        TelemetryPacket packet = new TelemetryPacket();

        // Color Sensor to detect sample in intake
        robot.colorSensor.enableLed(true);

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

        // OTOS Field Centric Drive Code
        drive.updatePoseEstimate();
        drive.setFieldCentricDrivePowers(
            new PoseVelocity2d(
                new Vector2d((gamepad1.left_stick_y), (gamepad1.left_stick_x)),
                gamepad1.right_stick_x),
                gamepad1.left_trigger,
                drive.pose.heading.toDouble()
        );

        //
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y,0);
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