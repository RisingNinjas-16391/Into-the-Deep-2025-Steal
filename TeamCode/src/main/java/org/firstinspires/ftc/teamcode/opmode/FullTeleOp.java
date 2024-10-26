package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositHighBasket;
import org.firstinspires.ftc.teamcode.subsystem.commands.transfer;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer = null;
    public ElapsedTime buttonTimer = null;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

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

        currentSample = robot.intake.sampleDetected();

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

        // Driving stuff
        // Minimum power of 0.2 and scale trigger value by remainder
        // Value to scale power to drivetrain based on driver trigger
        double speedMultiplier = (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.8) + 0.2;

        // For Mecanum:
        //setMecanumSpeeds(drive    r.getLeftX(), driver.getLeftY(), driver.getRightX(), speedMultiplier);
        //testSetMecanumSpeeds(driver, speedMultiplier);

        // Driver buttons
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.intake.setExtendoTarget(MAX_EXTENDO_EXTENSION);
        } else if ((driver.wasJustPressed(GamepadKeys.Button.B))) {
            robot.intake.setExtendoTarget(0);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            new transfer(robot.deposit, robot.intake);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
//            robot.imu.resetYaw();
        }

        if (driver.isDown(GamepadKeys.Button.DPAD_UP)) {
            robot.intake.setExtendoTarget(robot.extensionEncoder.getPosition() + 10); // Needs to be tested
        } else if (driver.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            robot.intake.setExtendoTarget(robot.extensionEncoder.getPosition() - 10); // Needs to be tested
        }

//        robot.intake.setPitchingIntake(robot.intake.stackHeight);

        // Operator buttons
        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            new depositHighBasket(robot.deposit);
        } else if (operator.wasJustPressed(GamepadKeys.Button.B)) {
            robot.deposit.setSlideTarget(0);
        }

        if ((operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) && buttonTimer.milliseconds() >= 200) {
            robot.deposit.wristIndex -= 1;
            robot.deposit.moveWrist();
            buttonTimer.reset();

        } else if ((operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) && buttonTimer.milliseconds() >= 200) {
            robot.deposit.wristIndex += 1;
            robot.deposit.moveWrist();
            buttonTimer.reset();
        }



        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.deposit.openClaw();
        } else if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.deposit.closeClaw();
        }

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}