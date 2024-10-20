package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.tuning.example.RobotOTOS;

@Photon
@TeleOp
public class TeleOpOTOS extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public SparkFunOTOSDrive drive;

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime buttonTimer = new ElapsedTime();
    public Rotation2d lastHeading;
    public double radsTurned = 0;
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
        lastHeading = Rotation2d.fromDouble(0);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        drive.updatePoseEstimate();

        radsTurned += drive.pose.heading.minus(lastHeading);
        lastHeading = drive.pose.heading;

        drive.setFieldCentricDrivePowers(new PoseVelocity2d(
            new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
            ),
            gamepad1.left_trigger,
            radsTurned
        );

        if (gamepad1.square) {
            radsTurned = 0;
        }

        telemetry.addData("otos", Math.toDegrees(radsTurned));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}