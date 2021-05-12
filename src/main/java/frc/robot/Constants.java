// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

import java.util.HashMap;

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


        public static final double kTrackwidthMeters = 0.55;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 2048;
        public static final double kGearRatio = 10.75;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                (kWheelDiameterMeters * Math.PI) / (kEncoderCPR * kGearRatio);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.696;
        public static final double kvVoltSecondsPerMeter = 1.13;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0956;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 5.56; //5.98
        public static final double kDDriveVel = 0.0010;
        public static final double kIDriveVel = 3.5; 
        public static final double robotLengthFeet = 35d / 12d;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kMaxSpeedMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    }

    public static final class LauncherConstants {
        public static final int Motor1CANID = 5;
        public static final int Motor2CANID = 6;
        public static final double kP = 0.75;
        public static final double kI = 0.28;
        public static final double kD = 0.07;
        public static final double speed = 10000;
        public static final double kV = 0.427;
        public static final double kA = 0.174;
        public static final double kEncoderUnitsPerRevolution = 4096;
        public static final double greenZoneSpeed = 28;
        public static final double yellowZoneSpeed = 14.78;
        public static final double blueZoneSpeed = 13.31;
        public static final double redZoneSpeed = 14.70;
        public static final double snowThrowerSpeed = 14.74;

    }

    public static final class LimelightConstants {
        public static final double limelightDistanceToGround = 1;
        public static final double powerPortDistanceToGround = 1;
        public static final double mountingAngle = Math.PI / 6;

    }

    public static final class PreLaunchConstants {
        public static final int Motor1CANID = 7;
    }

    public static final class IntakeConstants {
        public static final int MotorCANID = 8;
    }

    public static final class PixyConstants {
        public static final HashMap<String, Translation2d> PathCoordinates;
        static {
            PathCoordinates = new HashMap<String, Translation2d>();
            PathCoordinates.put("ARed", new Translation2d(0,0));
            PathCoordinates.put("ABlue", new Translation2d(0,0));
            PathCoordinates.put("BRed", new Translation2d(0,0));
            PathCoordinates.put("BBlue", new Translation2d(0,0));
        }
    }
}
