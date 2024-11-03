package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

@TeleOp
public class resetEncoders extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public SparkFunOTOSDrive drive;
    private FtcDashboard dash = FtcDashboard.getInstance();

    public ElapsedTime timer;
    public ElapsedTime buttonTimer;

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
        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        robot.liftEncoder.reset();
        robot.extensionEncoder.reset();

        telemetry.addData("liftEncoder", robot.liftEncoder.getPosition());
        telemetry.addData("extensionEncoder", robot.extensionEncoder.getPosition());

        // DO NOT REMOVE! Needed for telemetry
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}