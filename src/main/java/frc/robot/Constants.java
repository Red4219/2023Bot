package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class Constants {

    //
    // Drivetrain
    //
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.686;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.686;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(268);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(338);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(100);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(17);

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 1;
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND_SQUARED = 1;

    //
    // Arm
    //
    public static final int ARM_BASE_MOTOR_ID_1 = 13;
    public static final int ARM_BASE_MOTOR_ID_2 = 17;
    public static final int ARM_HIGH_MOTOR_ID_1 = 14;
    public static final int ARM_HIGH_MOTOR_ID_2 = 18;
    public static final int ARM_WRIST_MOTOR_ID_1 = 16;
    public static final int ARM_WRIST_MOTOR_ID_2 = 19;
    public static final int ARM_INTAKE_MOTOR_ID = 15;

    public static final double ARM_WRIST_MULTIPLIER = .5;
    public static final double ARM_HIGH_BAR_MULTIPLIER = .2;

    // ARM PRESET VALUES
    // ARM PRESET HIGH
    public static final double ARM_HIGH_ENCODER_VALUE = -11.0;
    public static final double ARM_HIGH_WRIST_ENCODER_VALUE = 1.5;
    public static final double ARM_HIGH_BASE_ENCODER_VALUE = 20.0;

    // ARM PRESET MID
    public static final double ARM_MID_ARM_ENCODER_VALUE = -8.0;
    public static final double ARM_MID_WRIST_ENCODER_VALUE = -0.2;
    //public static final double ARM_MID_BASE_ENCODER_VALUE = 9.0;
    public static final double ARM_MID_BASE_ENCODER_VALUE = 2.5;

    // ARM PRESET FOLD
    public static final double ARM_FOLD_ARM_ENCODER_VALUE = -2.3;
    public static final double ARM_FOLD_WRIST_ENCODER_VALUE = -0.9;
    public static final double ARM_FOLD_BASE_ENCODER_VALUE = -2.5;

    // ARM PRESET LOW
    public static final double ARM_LOW_ARM_ENCODER_VALUE = -2.0;
    public static final double ARM_LOW_WRIST_ENCODER_VALUE = 0.5;
    public static final double ARM_LOW_BASE_ENCODER_VALUE = 24.0;

    // ARM PRESET LOWLOW
    public static final double ARM_LOWLOW_ARM_ENCODER_VALUE = -2.0;
    public static final double ARM_LOWLOW_WRIST_ENCODER_VALUE = 0.5;
    public static final double ARM_LOWLOW_BASE_ENCODER_VALUE = 4.5;
    
    public enum IntakeDirection {
        IN, OUT
    }

    //
    // Buttons
    //

    public static final int DRIVER_BUTTON_A = 1;
    public static final int DRIVER_BUTTON_B = 2;
    public static final int DRIVER_BUTTON_X = 3;
    public static final int DRIVER_BUTTON_Y = 4;
    public static final int DRIVER_LEFT_BUMPER = 5;
    public static final int DRIVER_RIGHT_BUMPER = 6;
    public static final int DRIVER_BUTTON_SELECT = 7;
    public static final int DRIVER_BUTTON_START = 8;

    public static final int DRIVER_BUTTON_DISABLE_LED = DRIVER_BUTTON_A;
    public static final int DRIVER_BUTTON_RESET_GYRO = DRIVER_BUTTON_X;
    public static final int DRIVER_BUTTON_TOGGLE_FIELD_ORIENTED = DRIVER_BUTTON_Y;
    public static final int DRIVER_BUTTON_TOGGLE_LIMELIGHT_POSITION_CORRECTION = DRIVER_BUTTON_B;
    public static final int DRIVER_BUTTON_TOGGLE_LIMELIGHT_MODE = DRIVER_BUTTON_START;
    public static final int DRIVER_BUTTON_TOGGLE_AUTO_BALANCE = DRIVER_BUTTON_SELECT;

    public static final int OPERATOR_BUTTON_A = 1;
    public static final int OPERATOR_BUTTON_B = 2;
    public static final int OPERATOR_BUTTON_X = 3;
    public static final int OPERATOR_BUTTON_Y = 4;
    public static final int OPERATOR_LEFT_BUMPER = 5;
    public static final int OPERATOR_RIGHT_BUMPER = 6;
    public static final int OPERATOR_BUTTON_SELECT = 7;
    public static final int OPERATOR_BUTTON_START = 8;

    public static final int OPERATOR_BUTTON_HIGH = OPERATOR_BUTTON_Y;
    public static final int OPERATOR_BUTTON_MID = OPERATOR_BUTTON_B;
    public static final int OPERATOR_BUTTON_LOW = OPERATOR_BUTTON_A;
    public static final int OPERATOR_BUTTON_LOWLOW = OPERATOR_BUTTON_START;
    public static final int OPERATOR_BUTTON_FOLD = OPERATOR_BUTTON_X;

    public static double VISION_REFLECTIVE_THRESHOLD_MIN = -0.5;
    public static double VISION_REFLECTIVE_THRESHOLD_MAX  = 0.5;

    public static float PITCH_CHANGE_AMOUNT = 0.6f;
    public static float PITCH_CHANGE_THRESHOLD = 0.5f;
    public static float ROLL_CHANGE_AMOUNT = 0.1f;
    public static float ROLL_CHANGE_THRESHOLD = 0.5f;
}
