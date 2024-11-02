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
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

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
//

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            telemetry.addData("b", buttons);
            runningActions.add(new SequentialAction(
//                    new SleepAction(0.5),
//                    new InstantAction(() -> servo.setPosition(0.5))
            ));
        } else if (gamepad1.square && checkButton(gamepad1, "square")) {
            robot.intake.setWristIntake();
        } else if (gamepad1.circle && checkButton(gamepad1, "circle")) {
            robot.intake.setWristTransfer();
        }
        telemetry.update();

//        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
//            drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y,0);
//        }

        // Operator buttons

//        if (gamepad1.left_trigger > 0) {
//            robot.liftBottom.setPower(gamepad1.left_trigger);
//            robot.liftTop.setPower(gamepad1.left_trigger);
//        } else if (gamepad1.right_trigger > 0) {
//            robot.liftBottom.setPower(-gamepad1.right_trigger);
//            robot.liftTop.setPower(-gamepad1.right_trigger);
//        }
//        else {
//            robot.liftBottom.setPower(0);
//            robot.liftTop.setPower(0);
//        }

        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
            telemetry.addData("left trigger", GamepadKeys.Trigger.LEFT_TRIGGER);
        }


        if (gamepad1.left_trigger > 0) {
            telemetry.addData("sdk left trigger", gamepad1.left_trigger);
        }
        // Telemetry
        telemetry.addData("buttons", buttons);
        telemetry.update();

        // Runs RR Actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}