package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive.PARAMS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.lang.System;
import java.util.List;

public class Robot {
    public SolversMotor liftBottom;
    public SolversMotor liftTop;
    public SolversMotor extension;

    public SolversMotor frontLeftMotor;
    public SolversMotor frontRightMotor;
    public SolversMotor backLeftMotor;
    public SolversMotor backRightMotor;

    public SolversServo leftIntakePivot;
    public SolversServo rightIntakePivot;
    public SolversServo intakeClaw;
    public SolversServo trayServo;
    public SolversServo wrist;

    public SolversServo leftDepositPivot;
    public SolversServo rightDepositPivot;
    public SolversServo depositClaw;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder extensionEncoder;

    public SparkFunOTOSCorrected otos;

    public RevColorSensorV3 colorSensor;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;

    public Deposit deposit;
    public Intake intake;
    public SparkFunOTOSDrive drive;

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
        liftBottom = new SolversMotor((hardwareMap.get(DcMotor.class, "liftBottom")), 0.01);
        liftTop = new SolversMotor(hardwareMap.get(DcMotor.class, "liftTop"), 0.01);
        extension = new SolversMotor(hardwareMap.get(DcMotor.class, "extension"), 0.01);

        frontLeftMotor = new SolversMotor(hardwareMap.get(DcMotor.class, "FL"), 0.01);
        frontRightMotor = new SolversMotor(hardwareMap.get(DcMotor.class, "FR"), 0.01);
        backLeftMotor = new SolversMotor(hardwareMap.get(DcMotor.class, "BL"), 0.01);
        backRightMotor = new SolversMotor(hardwareMap.get(DcMotor.class, "BR"), 0.01);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftEncoder = new Motor(hardwareMap, "liftTop").encoder;
        extensionEncoder = new Motor(hardwareMap, "extension").encoder;

//        liftEncoder.setDirection(Motor.Direction.REVERSE);

        leftIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "leftIntakePivot"), 0.01);
        rightIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "rightIntakePivot"), 0.01);
        leftDepositPivot = new SolversServo(hardwareMap.get(Servo.class, "leftDepositPivot"), 0.01);
        rightDepositPivot = new SolversServo(hardwareMap.get(Servo.class, "rightDepositPivot"), 0.01);
        intakeClaw = new SolversServo(hardwareMap.get(Servo.class, "intakeClaw"), 0.01);
        depositClaw = new SolversServo(hardwareMap.get(Servo.class, "depositClaw"), 0.01);
        trayServo = new SolversServo(hardwareMap.get(Servo.class, "trayServo"), 0.01);
        wrist = new SolversServo(hardwareMap.get(Servo.class, "wrist"), 0.01);

        leftIntakePivot.setDirection(Servo.Direction.REVERSE);
        leftDepositPivot.setDirection(Servo.Direction.REVERSE);

        colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");

        otos = hardwareMap.get(SparkFunOTOSCorrected.class,"sensor_otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(PARAMS.offset);

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

        intake = new Intake();
        deposit = new Deposit();

        // OTOS stuff leave it alone
        drive = new SparkFunOTOSDrive(hardwareMap, startingPose);
        drive.calibrateOTOSimu();
        otos.setOffset(PARAMS.offset);
        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
        System.out.println(otos.setAngularScalar(PARAMS.angularScalar));

        if (opModeType.equals(OpModeType.AUTO)) {
            initHasMovement();
        }

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total

        // RR localizer note: It is technically possible to change the number of samples to slightly reduce init times,
        // however, I found that it caused pretty severe heading drift.
        // Also, if you're careful to always wait more than 612ms in init, you could technically disable waitUntilDone;
        // this would allow your OpMode code to run while the calibration occurs.
        // However, that may cause other issues.
        // In the future I hope to do that by default and just add a check in updatePoseEstimate for it
        System.out.println(otos.calibrateImu(255, true));
        System.out.println("OTOS calibration complete!");
    }

    public void initHasMovement() {
        deposit.init();
        intake.init();
    }

    public void getOTOSPosition() {
        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle
        SparkFunOTOSCorrected.Pose2D pos = otos.getPosition();
    }
}