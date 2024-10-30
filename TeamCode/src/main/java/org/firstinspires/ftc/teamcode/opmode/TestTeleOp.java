package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.commands.setIntake;
import org.firstinspires.ftc.teamcode.tuning.example.IntakeSubsystem;

@TeleOp
public class TestTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    private final IntakeSubsystem robot = IntakeSubsystem.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        super.reset();

        robot.init(hardwareMap);

        register(robot.deposit, robot.intake);

        // Initialize subsystems
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        robot.colorSensor.enableLed(true);

        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        telemetry.addData("RGB", "(" + red + ", " + green + ", " + blue + ")");

        telemetry.addData("distance (CM)", distance);


        if (distance < 3.5) {
            if (red >= green && red >= blue) {
                currentSample = SampleDetected.RED;
            } else if (green >= red && green >= blue) {
                currentSample = SampleDetected.YELLOW; // green means yellow sample
            } else if (distance >= 5.0) { // THIS IS FINE! It will get updated in loop
                currentSample = SampleDetected.BLUE; // Set default to be blue if on blue side, set default to be red if on red side.
            }
            else {
                currentSample = SampleDetected.YELLOW;
            }
        }

        switch (currentSample) {
            case RED:
                gamepad1.runLedEffect(SET_GAMEPAD_RED);
                gamepad2.runLedEffect(SET_GAMEPAD_RED);

            case BLUE:
                gamepad1.runLedEffect(SET_GAMEPAD_BLUE);
                gamepad2.runLedEffect(SET_GAMEPAD_BLUE);

            case YELLOW:
                gamepad1.runLedEffect(SET_GAMEPAD_YELLOW);
                gamepad2.runLedEffect(SET_GAMEPAD_YELLOW);

            case NONE:
                gamepad1.runLedEffect(SET_GAMEPAD_OFF);
                gamepad2.runLedEffect(SET_GAMEPAD_OFF);
        }

        // Runs the command scheduler and performs all the periodic() functions of each subsystem
        super.run();

        // Driver buttons
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            new setIntake(robot.intake, Intake.ExtendoState.IntakePivotState.READY_INTAKE);
        } else if ((driver.wasJustPressed(GamepadKeys.Button.B))) {
            new setIntake(robot.intake, Intake.ExtendoState.IntakePivotState.INTAKE);
        } else if ((driver.wasJustPressed(GamepadKeys.Button.X))) {
            new setIntake(robot.intake, Intake.ExtendoState.IntakePivotState.TRANSFER);
        } else if ((driver.wasJustPressed(GamepadKeys.Button.Y))) {
            new setIntake(robot.intake, Intake.ExtendoState.IntakePivotState.MIDDLE_HOLD);
        }

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}