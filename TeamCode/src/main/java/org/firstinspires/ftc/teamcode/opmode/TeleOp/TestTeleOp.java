package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected.BLUE;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected.RED;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SampleDetected.YELLOW;
import static org.firstinspires.ftc.teamcode.hardware.Globals.currentSample;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.startingPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositSafeRetracted;
import org.firstinspires.ftc.teamcode.subsystem.commands.realTransfer;
import org.firstinspires.ftc.teamcode.subsystem.commands.setDepositScoring;
import org.firstinspires.ftc.teamcode.subsystem.commands.setIntake;
import org.firstinspires.ftc.teamcode.subsystem.commands.testCommand;
import org.firstinspires.ftc.teamcode.subsystem.commands.transferReady;
import org.firstinspires.ftc.teamcode.subsystem.commands.transferToDeposit;

@TeleOp
public class TestTeleOp extends CommandOpMode {
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
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() ->
                        robot.drive.pose = new Pose2d(0, 0,0)));

        // All command testing stuff
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new transferReady(robot.deposit, robot.intake));

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new realTransfer(robot.deposit, robot.intake));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new setDepositScoring(robot.deposit, HIGH_SPECIMEN_HEIGHT));


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