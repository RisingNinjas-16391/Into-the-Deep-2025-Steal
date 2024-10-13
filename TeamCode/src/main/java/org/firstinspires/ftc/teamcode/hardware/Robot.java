package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DriveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.CoaxialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.drive.CoaxialSwerveModule;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversAxonServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

@Photon
public class Robot {
    public SolversMotor liftLeft;
    public SolversMotor liftRight;
    public SolversMotor extension;
    public SolversMotor intakeMotor;

    public SolversMotor frontLeftMotor;
    public SolversMotor frontRightMotor;
    public SolversMotor backLeftMotor;
    public SolversMotor backRightMotor;

    public SolversServo rightPivot;
    public SolversServo leftPivot;
    public SolversServo claw;
    public SolversServo wrist;

    public SolversAxonServo frontLeftServo;
    public SolversAxonServo frontRightServo;
    public SolversAxonServo backLeftServo;
    public SolversAxonServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder extensionEncoder;
    public Motor.Encoder parallelPod;
    public Motor.Encoder perpendicularPod;

    public CoaxialSwerveModule fL;
    public CoaxialSwerveModule fR;
    public CoaxialSwerveModule bL;
    public CoaxialSwerveModule bR;

    public RevColorSensorV3 colorSensor;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BHI260IMU imu;
    private double rawIMUAngle = 0;
    public static double imuOffset = 0;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;

    public Deposit deposit;
    public Intake intake;
    public CoaxialSwerveDrivetrain swerveDrivetrain;

    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) {
        liftLeft = new SolversMotor((hardwareMap.get(PhotonDcMotor.class, "liftLeft")), 0.01);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "liftRight"), 0.01);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "extension"), 0.01);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "intakeMotor"), 0.01);

        frontLeftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "frontLeftMotor"), 0.01);
        frontRightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "frontRightMotor"), 0.01);
        backLeftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "backLeftMotor"), 0.01);
        backRightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "backRightMotor"), 0.01);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivot = new SolversServo(hardwareMap.get(PhotonServo.class, "leftClaw"), 0.0);
        rightPivot = new SolversServo(hardwareMap.get(PhotonServo.class, "rightClaw"), 0.0);
        claw = new SolversServo(hardwareMap.get(PhotonServo.class, "rightArm"), 0.0);
        wrist = new SolversServo(hardwareMap.get(PhotonServo.class, "wrist"), 0.0);

        frontLeftServo = new SolversAxonServo(hardwareMap.get(PhotonCRServo.class, "frontLeftServo"), 0.01);
        frontRightServo = new SolversAxonServo(hardwareMap.get(PhotonCRServo.class, "frontRightServo"), 0.01);
        backLeftServo = new SolversAxonServo(hardwareMap.get(PhotonCRServo.class, "backLeftServo"), 0.01);
        backRightServo = new SolversAxonServo(hardwareMap.get(PhotonCRServo.class, "backRightServo"), 0.01);

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivot.setDirection(Servo.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        frontLeftServo.setServoEncoder(frontLeftEncoder);
        frontRightServo.setServoEncoder(frontRightEncoder);
        backLeftServo.setServoEncoder(backLeftEncoder);
        backRightServo.setServoEncoder(backRightEncoder);

        liftEncoder = new MotorEx(hardwareMap, "liftRight").encoder;
        extensionEncoder = new MotorEx(hardwareMap, "extension").encoder;

        parallelPod = new MotorEx(hardwareMap, "backRightMotor").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        perpendicularPod = new MotorEx(hardwareMap, "backLeftMotor").encoder;
        perpendicularPod.setDirection(Motor.Direction.REVERSE);

        colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");

        // IMU orientation
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Bulk reading enabled!
        // AUTO mode will bulk read by default and will redo and clear cache once the exact same read is done again
        // MANUAL mode will bulk read once per loop but needs to be manually cleared
        // Also in opModes only clear ControlHub cache as it is a hardware write
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }
        }

        imuOffset = Globals.STARTING_HEADING;

        fL = new CoaxialSwerveModule(frontLeftMotor, frontLeftServo, -2.524);
        fR = new CoaxialSwerveModule(frontRightMotor, frontRightServo, 0.529);
        bL = new CoaxialSwerveModule(backLeftMotor, backLeftServo, -2.163);
        bR = new CoaxialSwerveModule(backRightMotor, backRightServo, -1.832);

        fL.init();
        fR.init();
        bL.init();
        bR.init();

        swerveDrivetrain = new CoaxialSwerveDrivetrain(new CoaxialSwerveModule[]{ fL, fR, bL, bR });

        intake = new Intake();
        deposit = new Deposit();

        // Inits commented out for kinematics testing
//        deposit.init();
//        intake.init();

        // Add any OpMode specific initializations here
        if (Globals.opModeType == OpModeType.AUTO) {
            // deposit.initAuto();
        } else {
            // deposit.initTeleOp();
        }
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            Thread imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        rawIMUAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    }
                }
            });
            imuThread.start();
        }
    }

    public void resetIMU() {
        imuOffset = rawIMUAngle;
    }

    public double getAngle() {
        return rawIMUAngle - imuOffset;
    }

    public static void setMecanumSpeeds(double leftX, double leftY, double rightX, double speedMultiplier) {
        if (driveMode.equals(DriveMode.FIELD_CENTRIC)) {
            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-instance.getAngle()) + leftY * Math.sin(-instance.getAngle());
            double rotY = leftX * Math.sin(-instance.getAngle()) - leftY * Math.cos(-instance.getAngle());

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);

            double frontLeftPower = ((rotY + rotX + rightX) / denominator) * speedMultiplier;
            double backLeftPower = ((rotY - rotX + rightX) / denominator) * speedMultiplier;
            double frontRightPower = ((rotY - rotX - rightX) / denominator) * speedMultiplier;
            double backRightPower = ((rotY + rotX - rightX) / denominator) * speedMultiplier;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);

        } else {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(leftY) + Math.abs((leftX * 1.1)) + Math.abs(rightX), 1);

            double frontLeftPower = (leftY + (leftX * 1.1) + rightX) / denominator;
            double backLeftPower = (leftY - (leftX * 1.1) + rightX) / denominator;
            double frontRightPower = (leftY - (leftX * 1.1) - rightX) / denominator;
            double backRightPower = (leftY + (leftX * 1.1) - rightX) / denominator;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);
        }
    }

    public static void testSetMecanumSpeeds(GamepadEx gamepad, double speedMultiplier) {
        double leftY = gamepad.getLeftY(); // Remember, Y stick value is reversed
        double leftX = gamepad.getLeftX(); // Counteract imperfect strafing
        double rightX = gamepad.getRightX();

        if (driveMode.equals(DriveMode.FIELD_CENTRIC)) {
            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-instance.getAngle()) + leftY * Math.sin(-instance.getAngle());
            double rotY = leftX * Math.sin(-instance.getAngle()) - leftY * Math.cos(-instance.getAngle());

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);

            double frontLeftPower = ((rotY + rotX + rightX) / denominator) * speedMultiplier;
            double backLeftPower = ((rotY - rotX + rightX) / denominator) * speedMultiplier;
            double frontRightPower = ((rotY - rotX - rightX) / denominator) * speedMultiplier;
            double backRightPower = ((rotY + rotX - rightX) / denominator) * speedMultiplier;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);

        } else {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(leftY) + Math.abs((leftX * 1.1)) + Math.abs(rightX), 1);

            double frontLeftPower = (leftY + (leftX * 1.1) + rightX) / denominator;
            double backLeftPower = (leftY - (leftX * 1.1) + rightX) / denominator;
            double frontRightPower = (leftY - (leftX * 1.1) - rightX) / denominator;
            double backRightPower = (leftY + (leftX * 1.1) - rightX) / denominator;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);
        }
    }
}