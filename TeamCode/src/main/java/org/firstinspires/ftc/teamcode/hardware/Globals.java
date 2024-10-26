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

    public static SampleDetected currentSample = SampleDetected.YELLOW;

    public static DriveMode driveMode;

    public static Gamepad.LedEffect SET_GAMEPAD_RED = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS)
            .build();

    public static Gamepad.LedEffect SET_GAMEPAD_YELLOW = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS)
            .build();

    public static Gamepad.LedEffect SET_GAMEPAD_BLUE = new Gamepad.LedEffect.Builder()
            .addStep(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS)
            .build();

    public static Gamepad.LedEffect SET_GAMEPAD_OFF = new Gamepad.LedEffect.Builder()
            .addStep(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS)
            .build();


    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // By default values refer to servo positions, unless otherwise specified
    // By default for values that control opposite running hardware, the right value of the hardware is used
    // e.g. for ARM_TRANSFER_POS, it should correspond with the real position of the servo at the transfer

    // Deposit
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.0;
    public static double DEPOSIT_PIVOT_SCORING_POS = 1.0;

    // Intake
    public static double WRIST_TRANSFER_POS = 0;
    public static double WRIST_INTAKE_POS = 0;
    public static double INTAKE_PIVOT_TRANSFER_POS = 0;
    public static double INTAKE_PIVOT_HOLD_POS = 0;
    public static double INTAKE_PIVOT_PICKUP_POS = 0;

    // 4th item or 3rd with 0-index is always the default (middle horizontal)

    public static double[] WRIST_POSITIONS = {1, 0.82, 0.64, 0.46, 0.28, 0.08}; // TODO: TUNE
    public static double INTAKE_CLAW_OPEN_POS = 0.07;
    public static double INTAKE_CLAW_CLOSE_POS = 0.18;

    public static double DEPOSIT_CLAW_OPEN_POS = 0.53;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.4;

    // TODO: TUNE ALL OF THESE
    // Extendo + Slides
    public static double MAX_EXTENDO_EXTENSION = 10000; // Encoder ticks
    // Encoder ticks for max extension on scoring slides
    public static double MAX_SLIDES_EXTENSION = 10000; // Encoder ticks
    // Encoder ticks for first pixel row height
    public static double LOW_BUCKET_HEIGHT = 0; // Encoder ticks
    public static double HIGH_BUCKET_HEIGHT = 0; // Encoder ticks
    public static double LOW_SPECIMEN_HEIGHT = 0; // Encoder ticks
    public static double HIGH_SPECIMEN_HEIGHT = 0; // Encoder ticks
    public static double AUTO_HANG_HEIGHT = 0; // Encoder ticks
    public static double ENDGAME_HANG_HEIGHT = 0; // Encoder ticks
}