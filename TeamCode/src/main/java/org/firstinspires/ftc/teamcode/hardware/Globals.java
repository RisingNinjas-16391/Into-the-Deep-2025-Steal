package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class Globals {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public static OpModeType opModeType;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public enum SampleDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    public static SampleDetected currentSample = SampleDetected.NONE;

    public static DriveMode driveMode;

    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // By default values refer to servo positions, unless otherwise specified
    // By default for values that control opposite running hardware, the right value of the hardware is used
    // e.g. for ARM_TRANSFER_POS, it should correspond with the real position of the servo at the transfer

    // TODO: TUNE. 9.99 or other sus numbers generally means not tuned!

    // Intake

    // wrist perpendicular to tray pos: 0.53

    public static double WRIST_TRANSFER_POS = 0.80;
    public static double WRIST_INTAKE_POS = 0.53;
    public static double[] WRIST_POSITIONS = {1, 9.99, 9.99, 0.53, 9.99, 0.0}; // TODO: TUNE. Note that 4th item or 3rd with 0-index is always the default (intake pos)
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.32;
    public static double INTAKE_PIVOT_HOLD_POS = 0.4;
    public static double INTAKE_PIVOT_PICKUP_POS = 0.53;
    public static double INTAKE_PIVOT_READY_PICKUP_POS = 0.5;
    public static double INTAKE_CLAW_OPEN_POS = 0.65;
    public static double INTAKE_CLAW_CLOSE_POS = 0.85;
    public static double TRAY_OPEN_POS = 1.0;
    public static double TRAY_CLOSE_POS = 0.35;

    // Deposit
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.0;
    public static double DEPOSIT_PIVOT_SCORING_POS = 1.0;
    public static double DEPOSIT_CLAW_OUTSIDE_OPEN_POS = 0.625;
    public static double DEPOSIT_CLAW_OUTSIDE_CLOSE_POS = 0.83;
    public static double DEPOSIT_CLAW_INSIDE_OPEN_POS = 0.88;
    public static double DEPOSIT_CLAW_INSIDE_CLOSE_POS = 0.6;

    // TODO: TUNE ALL OF THESE
    // Extendo
    public static double MAX_EXTENDO_EXTENSION = 10000; // Encoder ticks
    public static double AUTON_EXTENDO_EXTENSION; // Encoder ticks

    // Slides
    public static double MAX_SLIDES_EXTENSION = 10000; // Encoder ticks
    public static double SLIDES_PIVOT_READY_EXTENSION = 10000; // Encoder ticks
    public static double LOW_BUCKET_HEIGHT = 775; // Encoder ticks
    public static double HIGH_BUCKET_HEIGHT = 1500; // Encoder ticks

//    public static double LOW_SPECIMEN_HEIGHT = 0; // Encoder ticks
//    public static double LOW_SPECIMEN_ATTACH_HEIGHT = 0; // Encoder ticks
    public static double HIGH_SPECIMEN_HEIGHT = 580; // Encoder ticks
    public static double HIGH_SPECIMEN_ATTACH_HEIGHT = 400; // Encoder ticks
    public static double AUTO_HANG_HEIGHT = 0; // Encoder ticks
    public static double ENDGAME_HANG_HEIGHT = 0; // Encoder ticks
}