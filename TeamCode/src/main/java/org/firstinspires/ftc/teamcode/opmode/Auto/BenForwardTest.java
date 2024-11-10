package org.firstinspires.ftc.teamcode.opmode.Auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositSafeRetracted;

@Config
@Autonomous
public class BenForwardTest extends OpMode {
    private final Robot robot = Robot.getInstance();
    public static int index = 0;
    public static double motorSpeeds = 0.5;
    public static int stopTimer = 2200;

    public ElapsedTime timer;

    @Override
    public void init() {
        opModeType = OpModeType.AUTO;
        startingPose = new Pose2d(0, 0, 0);

        CommandScheduler.getInstance().enable();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.deposit, robot.intake);

    }

    @Override
    public void loop() {
        if (timer == null) {

            timer = new ElapsedTime();


            robot.drive.leftBack.setPower(motorSpeeds);
            robot.drive.leftFront.setPower(motorSpeeds);
            robot.drive.rightBack.setPower(motorSpeeds);
            robot.drive.rightFront.setPower(motorSpeeds);

            robot.colorSensor.enableLed(true);
            if (index == 0) {
                index = 1;
            }
        }

        telemetry.addData("index", index);
        telemetry.addData("liftBottom power", robot.liftBottom.getPower());
        telemetry.addData("liftTop power", robot.liftTop.getPower());
        telemetry.update();







        CommandScheduler.getInstance().run();
    }
}