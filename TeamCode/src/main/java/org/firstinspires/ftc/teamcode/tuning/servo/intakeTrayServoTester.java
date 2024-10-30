package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DriveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.System.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CLAW_SERVO_POS;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.LEFT_SERVO_POS;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.WRIST_SERVO_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tuning.example.IntakeSubsystem;

@Config
@TeleOp
public class intakeTrayServoTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final IntakeSubsystem robot = IntakeSubsystem.getInstance();
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intakeClaw.setPosition(CLAW_SERVO_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD) {
            robot.intakeClaw.setPosition(CLAW_SERVO_POS);
            robot.wrist.setPosition(WRIST_SERVO_POS);
            robot.leftIntakePivot.setPosition(LEFT_SERVO_POS);
            robot.rightIntakePivot.setPosition(LEFT_SERVO_POS);
        }

        CLAW_SERVO_POS = Math.max(Math.min(CLAW_SERVO_POS, 1), 0);
        WRIST_SERVO_POS = Math.max(Math.min(WRIST_SERVO_POS, 1), 0);
        LEFT_SERVO_POS = Math.max(Math.min(LEFT_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        robot.colorSensor.enableLed(true);

        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        telemetry.addData("RGB", "(" + red + ", " + green + ", " + blue + ")");

        telemetry.addData("distance (CM)", distance);

        String color = "";

        if (distance < 3.5) {
            if (red >= green && red >= blue) {
                color = "red";
            } else if (green >= red && green >= blue) {
                color = "green";
            } else {
                color = "blue"; // Set default to be blue if on blue side, set default to be red if on red side.
            }
        }

        telemetry.addData("color", color);

        telemetry.addData("intakeClaw getPosition", robot.intakeClaw.getPosition());
        telemetry.addData("intakeClaw", round(CLAW_SERVO_POS, 2));

        telemetry.addData("intakeClaw getPosition", robot.intakeClaw.getPosition());
        telemetry.addData("intakeClaw", round(CLAW_SERVO_POS, 2));

        telemetry.addData("intakeClaw getPosition", robot.intakeClaw.getPosition());
        telemetry.addData("intakeClaw", round(CLAW_SERVO_POS, 2));

        telemetry.addData("wrist getPosition", robot.wrist.getPosition());
        telemetry.addData("wrist", round(WRIST_SERVO_POS, 2));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}