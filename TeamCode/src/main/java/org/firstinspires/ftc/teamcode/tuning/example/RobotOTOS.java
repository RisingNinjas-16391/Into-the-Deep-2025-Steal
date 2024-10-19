package org.firstinspires.ftc.teamcode.tuning.example;

import static org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive.PARAMS;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

@Photon
public class RobotOTOS {
//    public SolversServo leftServo;
//    public SolversServo rightServo;
//    public SolversServo centerServo;
//    public SolversMotor centerMotor;
//    public SolversMotor leftMotor;
//    public SolversMotor rightMotor;
    public SparkFunOTOS otos;


    private static RobotOTOS instance = null;
    public boolean enabled;

    public static RobotOTOS getInstance() {
        if (instance == null) {
            instance = new RobotOTOS();
        }
        instance.enabled = true;
        return instance;
    }

    public List<LynxModule> allHubs;
    public LynxModule ControlHub;

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) { // CONFIG: robotTester
//        leftServo = new SolversServo(hardwareMap.get(PhotonServo.class, "leftServo"), 0.0); // Servo Slot 4 on Control Hub
//        rightServo = new SolversServo(hardwareMap.get(PhotonServo.class, "rightServo"), 0.0); // Servo Slot 5 on Control Hub
//        centerServo = new SolversServo(hardwareMap.get(PhotonServo.class, "centerServo"), 0.0); // Servo Slot 0 on Control Hub
//
//        leftServo.setDirection(Servo.Direction.REVERSE);
//
//        centerMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "centerMotor"), 0.01); // Motor Slot 0 on Control Hub
//
//        leftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "leftMotor"), 0.01); // Motor Slot 2 on Control Hub
//
//        rightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "rightMotor"), 0.01); // Motor Slot 3 on Control Hub
//
//        centerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Add any OpMode specific initializations here
        if (Globals.opModeType == Globals.OpModeType.AUTO) {
            // deposit.initAuto();
        }

        else {
//            otos.setOffset(PARAMS.offset);
//            otos.setLinearScalar(PARAMS.linearScalar);
//            otos.setAngularScalar(PARAMS.angularScalar);
//            otos.calibrateImu(833, true);

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
        }
    }
}
