package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Globals {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public static OpModeType opModeType;

    public static Pose2d startingPose = new Pose2d(0, 0, 0);

    public enum SampleDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    public static SampleDetected currentSample = SampleDetected.NONE;

    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // By default values refer to servo positions, unless otherwise specified
    // By default for values that control opposite running hardware, the right value of the hardware is used
    // e.g. for ARM_TRANSFER_POS, it should correspond with the real position of the servo at the transfer

    // TODO: TUNE. 9.99 or other sus numbers (like 10,000) generally means not tuned!

    // Intake

    // wrist perpendicular to tray pos: 0.53

    public static double WRIST_INNER_TRANSFER_POS = 0.25;
    public static double WRIST_OUTER_TRANSFER_POS = 0.52;
    public static double WRIST_INTAKE_POS = 0.52;
    public static double[] WRIST_POSITIONS = {0, 0.125, 0.25, 0.4, 0.52}; // TODO: TUNE. Note that 3rd item or 2nd with 0-index is always the default (intake pos)
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.32;
    public static double INTAKE_PIVOT_HOLD_POS = 0.4;
    public static double INTAKE_PIVOT_PICKUP_POS = 0.525;
    public static double INTAKE_PIVOT_READY_PICKUP_POS = 0.48;
    public static double INTAKE_CLAW_INNER_OPEN_POS = 0.85;
    public static double INTAKE_CLAW_INNER_CLOSE_POS = 0.6;
    public static double INTAKE_CLAW_OUTER_OPEN_POS = 0.6;
    public static double INTAKE_CLAW_OUTER_CLOSE_POS = 0.85;
    public static double TRAY_OPEN_POS = 1.0;
    public static double TRAY_CLOSE_POS = 0.35;

    // Deposit
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.07;
    public static double DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS = 0.85;
    public static double DEPOSIT_PIVOT_SCORING_POS = 1.0;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.32;
    public static double DEPOSIT_CLAW_OUTSIDE_OPEN_POS = 0.625;
    public static double DEPOSIT_CLAW_OUTSIDE_CLOSE_POS = 0.4;


    // TODO: TUNE ALL OF THESE
    // Extendo
    public static double MAX_EXTENDO_EXTENSION = 480; // Encoder ticks
    public static double AUTON_EXTENDO_EXTENSION; // Encoder ticks

    // Slides
    public static double MAX_SLIDES_EXTENSION = 1800; // Encoder ticks
    public static double SLIDES_PIVOT_READY_EXTENSION = 200; // Encoder ticks
    public static double LOW_BUCKET_HEIGHT = 900; // Encoder ticks
    public static double HIGH_BUCKET_HEIGHT = 1800; // Encoder ticks

    public static double SPECIMEN_INTAKE_HEIGHT = 0;

//    public static double LOW_SPECIMEN_HEIGHT = 0; // Encoder ticks
//    public static double LOW_SPECIMEN_ATTACH_HEIGHT = 0; // Encoder ticks
    public static double HIGH_SPECIMEN_HEIGHT = 600; // Encoder ticks
    public static double HIGH_SPECIMEN_ATTACH_HEIGHT = 400; // Encoder ticks
    public static double AUTO_HANG_HEIGHT = 700; // Encoder ticks
    public static double ENDGAME_HANG_HEIGHT = 800; // Encoder ticks
}