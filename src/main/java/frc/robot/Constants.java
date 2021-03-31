// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;


        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final double kTrackwidthMeters = 0.55;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 2048;
        public static final double kGearRatio = 10.75;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) (kEncoderCPR * kGearRatio);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.696;
        public static final double kvVoltSecondsPerMeter = 1.13;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0297;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.9;
        public static final double kDDriveVel = 0;
        public static final double robotLengthFeet = 35 / 12;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static double kMaxSpeedMetersPerSecond = 1.5;
        public static double kMaxAccelerationMetersPerSecondSquared = 1;
    }

    public static final class InterstellarAccuracyConstants {
        public static Translation2d GreenZoneFeet = new Translation2d(10 - DriveConstants.robotLengthFeet / 2, 7.5);
        public static Translation2d YellowZoneFeet = new Translation2d(15 - DriveConstants.robotLengthFeet / 2, 7.5);
        public static Translation2d BlueZoneFeet = new Translation2d(20 - DriveConstants.robotLengthFeet / 2, 7.5);
        public static Translation2d RedZoneFeet = new Translation2d(25 - DriveConstants.robotLengthFeet / 2, 7.5);
        public static Translation2d ReintroductionZone = new Translation2d(25 + DriveConstants.robotLengthFeet / 2, 7.5);
    }

    public static final class LauncherConstants {
        public static final int Motor1CANID = 5;
        public static final int Motor2CANID = 6;
        public static final double kP = 0.7;
        public static final double kI = 0.001;
        public static final double kD = 60;
        public static final double speed = 10000;
        public static final double kEncoderUnitsPerRevolution = 100;
    }

    public static final class LimelightConstants {
        public static final double limelightDistanceToGround = 1;
        public static final double powerPortDistanceToGround = 1;
        public static final double mountingAngle = Math.PI / 6;

    }

    public static final class PreLaunchConstants {
        public static final int Motor1CANID = 7;
        public static final double SpeedMetersPerSecond = 2;
    }

    public static final class IntakeConstants {
        public static final int MotorCANID = 8;
    }
}
