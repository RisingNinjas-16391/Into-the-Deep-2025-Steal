package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp
@Config
public class TestTeleOp extends CommandOpMode {
    public static double motorSpeed = 0.0;
    Intake.ExtendoState.IntakePivotState intakePivotState;
    ElapsedTime wristTimer = new ElapsedTime();
    public String moveWrist = "";
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        super.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);

        register(robot.deposit, robot.intake);

        wristTimer.reset();
    }

    @Override
    public void run() {

        // Runs the command scheduler and performs all the periodic() functions of each subsystem
        super.run();

        currentSample = robot.intake.sampleDetected();

        telemetry.addData("currentSample", currentSample.toString());
        telemetry.addData("distance", robot.intake.colorSensorDistance());


//        gamepad1.runLedEffect(SET_GAMEPAD_YELLOW);
//        gamepad2.runLedEffect(SET_GAMEPAD_YELLOW);
//
//        switch (currentSample) {
//            case RED:
//                gamepad1.runLedEffect(SET_GAMEPAD_RED);
//                gamepad2.runLedEffect(SET_GAMEPAD_RED);
//                break;
//            case BLUE:
//                gamepad1.runLedEffect(SET_GAMEPAD_BLUE);
//                gamepad2.runLedEffect(SET_GAMEPAD_BLUE);
//                break;
//            case YELLOW:
//                gamepad1.runLedEffect(SET_GAMEPAD_YELLOW);
//                gamepad2.runLedEffect(SET_GAMEPAD_YELLOW);
//                break;
//            case NONE:
//                gamepad1.runLedEffect(SET_GAMEPAD_OFF);
//                gamepad2.runLedEffect(SET_GAMEPAD_OFF);
//                break;
//        }

        // Driver buttons
        if (gamepad1.triangle) {
            robot.intake.setPivotServo(INTAKE_PIVOT_READY_PICKUP_POS);
            wristTimer.reset();
            moveWrist = "intake";
        }

        else if (gamepad1.cross) {
            robot.intake.setPivotServo(INTAKE_PIVOT_PICKUP_POS);
            wristTimer.reset();
            moveWrist = "intake";
        }

        else if (gamepad1.circle) {
            robot.intake.closeClaw();
            robot.intake.setPivotServo(INTAKE_PIVOT_HOLD_POS);
        }

        else if (gamepad1.square) {
            robot.intake.closeClaw();
            robot.intake.setPivotServo(INTAKE_PIVOT_TRANSFER_POS);

            wristTimer.reset();
            moveWrist = "transfer";
        }

        if (gamepad1.left_stick_button) {
            robot.intake.openClaw();

        } else if (gamepad1.right_stick_button) {
            robot.intake.closeClaw();
        }

        if (moveWrist.equals("intake") && wristTimer.milliseconds() >= 1000) {
            robot.intake.setWristIntake();
            robot.intake.openClaw();
            moveWrist = "";
        }

        if (moveWrist.equals("transfer") && wristTimer.milliseconds() >= 1000) {
            robot.intake.openClaw();
            moveWrist = "";
        }

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

        telemetry.addData("lt", gamepad1.left_trigger);
        telemetry.addData("rt", gamepad1.right_trigger);

        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}