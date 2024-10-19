package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive.PARAMS;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversAxonServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.lang.System;
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

    public SparkFunOTOSCorrected otos;

    public RevColorSensorV3 colorSensor;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;

    public Deposit deposit;
    public Intake intake;

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

        // Add any OpMode specific initializations here
        if (Globals.opModeType == OpModeType.AUTO) {
            // deposit.initAuto();
        }

        else {
            otos.setOffset(PARAMS.offset);
            System.out.println("OTOS calibration beginning!");
            System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
            System.out.println(otos.setAngularScalar(PARAMS.angularScalar));

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
    }

    public void getOTOSPosition() {
        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle
        SparkFunOTOSCorrected.Pose2D pos = otos.getPosition();
    }
}