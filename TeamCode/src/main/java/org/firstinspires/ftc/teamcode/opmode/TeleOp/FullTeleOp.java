package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public SparkFunOTOSDrive drive;

    public ElapsedTime timer = null;
    public ElapsedTime buttonTimer = null;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;
    String buttons = "";



    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);
        robot.deposit.initTeleOp();

        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));
        drive.calibrateOTOSimu();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        // Only run first time, but not in init because IMU thread would start polling?
        // TO-DO: Need to check if this is needed or if it is safe to put in initialize()
        if (timer == null) {
            timer = new ElapsedTime();
            buttonTimer = new ElapsedTime();
        }
        // Endgame/hang rumble after 110 seconds to notify driver to hang
        else if ((timer.seconds() > 110) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        super.run();

        currentSample = robot.intake.sampleDetected();

        try {
            buttons = String.valueOf(gamepad1).substring(75).substring(1);
        }
        catch (Exception ignored) {
            assert true;
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

        drive.setFieldCentricDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                (gamepad1.left_stick_y),
                                (gamepad1.left_stick_x)
                        ),
                        gamepad1.right_stick_x
                ),
                gamepad1.left_trigger,
                drive.pose.heading.toDouble()
        );

        // Driver buttons
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            assert true;
        } else if ((driver.wasJustPressed(GamepadKeys.Button.B))) {
            assert true;
        } else if ((driver.wasJustPressed(GamepadKeys.Button.X))) {
            assert true;
        } else if ((driver.wasJustPressed(GamepadKeys.Button.Y))) {
            assert true;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y,0);
        }

        // Operator buttons

        // Telemetry
        telemetry.addData("buttons", buttons);
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}