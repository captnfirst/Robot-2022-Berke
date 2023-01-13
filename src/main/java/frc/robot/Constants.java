// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int LEFT_MASTER_MOTOR_PWM = 0;
        public static final int RIGHT_MASTER_MOTOR_PWM = 1;

        // follow vision target cmd speeds
        public static final double TARGET_FOLLOWING_SPEED = 0.5;
        public static final double BUTTERY_FOLLOWING_SPEED = 0.035;
    }

    public static final class JoystickConstants {
        public static final int JOYSTICK_PORT = 0;

        public static final int ARCADE_DRIVE_SPEED_AXIS = 1;
        public static final int ARCADE_DRIVE_TURN_AXIS = 0;

        public static final int ARCADE_DRIVE_REVERSE_BTN = 7;
        public static final int ARCADE_DRIVE_INVERT_BTN = 8;

        public static final int INTAKE_BTN = 6;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 7;
        public static final double INTAKE_SLEW = 5;

        // intake arm
        public static final int ARM_MOTOR_PWM = 2;
    }

    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 8;
    }

    public static final class TriggerConstants {
        public static final int TRIGGER_MOTOR_PWM = 3;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID = 10;

        public static final int CAN_TIMEOUT = 10;
        public static final double GEAR_RATIO = 84.0 / 54.0;
        public static final double TALON_100MS_IN_1S = 10.0;
    }

    public static final class HoodConstants {
        public static final int HOOD_MOTOR_PWM = 5;

        public static final byte channelA = 0; // DIO 0
        public static final byte channelB = 1; // DIO 1
        public static final boolean reverseDirection = false;
    }

    public static final class ClimberConstants {
        public static final int FIXED_CLIMBER_MOTOR_PWM = 4;
        public static final int LEFT_CLIMBER_MOTOR_ID = 5;
        public static final int RIGHT_CLIMBER_MOTOR_ID = 6;
    }

    public static final class Vision {
        public static final double FIELD_WIDTH = 8.23; // Alanın genişliği. [m]
        public static final double FIELD_LENGTH = 16.46; // Alanın uzunluğu. [m]

        public static final double CAMERA_HEIGHT = 0.767; // kameranın yüksekliği[m]
        public static final double TARGET_HEIGHT_FROM_GROUND = 2.64; // yerden hedefin yüksekliği [m]
        public static final double CAMERA_PITCH = 30; // Kameranın Bakış Açısı. [deg]
        public static final double DIAG_FOV = 75; // Bakış Açısı Saha. [deg]
        public static final int CAM_RESOLUTION_WIDTH = 640; // Kamera çözünürlüğünün genişliği. [pixel]
        public static final int CAM_RESOLUTION_HEIGHT = 480; // Kamera çözünürlüğünün yüksekliği. [pixel]
        public static final double MIN_TARGET_AREA = 10; // Minimum hedef alanı. [pixel^2]
        public static final double TARGET_WIDTH = 1.36; // Görüş hedef şeridinin genişliği. [m]
        public static final double TARGET_HEIGHT = 0.05; // Görüş hedef şeridinin yüksekliği. [m]

        // Hub'ın alana göre konumu.
        public static final Pose2d HUB_POSE = new Pose2d(new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2),
                new Rotation2d());
        // Robota göre görüntü işleme kamerasının konumu.
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(new Translation2d(0.038, 0.171),
                new Rotation2d());
    }

}
