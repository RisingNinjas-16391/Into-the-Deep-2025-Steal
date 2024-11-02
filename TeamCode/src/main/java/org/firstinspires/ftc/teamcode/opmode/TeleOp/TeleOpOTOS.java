package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.tuning.example.RobotOTOS;

@TeleOp
public class TeleOpOTOS extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public SparkFunOTOSDrive drive;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime buttonTimer = new ElapsedTime();
    private final RobotOTOS robot = RobotOTOS.getInstance();

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        super.reset();

        robot.init(hardwareMap);

        timer.reset();
        buttonTimer.reset();

        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));
        drive.calibrateOTOSimu();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
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

        if (gamepad1.square) {
            drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y,0);
        }

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}