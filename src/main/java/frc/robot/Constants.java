package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class Constants {
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

    public static final int DRIVER_BUTTON_A = 1;
    public static final int DRIVER_BUTTON_B = 2;
    public static final int DRIVER_BUTTON_X = 3;
    public static final int DRIVER_BUTTON_Y = 4;
    public static final int DRIVER_LEFT_BUMPER = 5;
    public static final int DRIVER_RIGHT_BUMPER = 6;

    public static final int DRIVER_BUTTON_DISABLE_LED = DRIVER_BUTTON_A;
    public static final int DRIVER_BUTTON_RESET_GYRO = DRIVER_BUTTON_X;
    public static final int DRIVER_BUTTON_TOGGLE_FIELD_ORIENTED = DRIVER_BUTTON_Y;
    public static final int DRIVER_BUTTON_TOGGLE_LIMELIGHT_POSITION_CORRECTION = DRIVER_BUTTON_B;

    public static final int OPERATOR_BUTTON_A = 1;
    public static final int OPERATOR_BUTTON_B = 2;
    public static final int OPERATOR_BUTTON_X = 3;
    public static final int OPERATOR_BUTTON_Y = 4;
    public static final int OPERATOR_LEFT_BUMPER = 5;
    public static final int OPERATOR_RIGHT_BUMPER = 6;   
}
