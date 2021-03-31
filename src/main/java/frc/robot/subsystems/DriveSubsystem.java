// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lib.MotorFaultLogger;
import lib.CustomDifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.util.ArrayList;
import java.util.Collection;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final WPI_TalonFX m_leftMotors = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
    private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
    // The motors on the right side of the drive.
    private final WPI_TalonFX m_rightMotors = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    // The left-side drive encoder
    private final WPI_TalonFX m_leftEncoder = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
    // The right-side drive encoder
    private final WPI_TalonFX m_rightEncoder = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    public Orchestra _Orchestra;
    // The gyro sensor
    public AHRS navX = new AHRS(SPI.Port.kMXP);
    public final Gyro m_gyro = navX;
    Collection<TalonFX> _fxes = new ArrayList<TalonFX>();
    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
    NetworkTableEntry m_thetaEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("theta");
    Faults leftMasterFaults;
    Faults rightMasterFaults;
    Faults leftFollowerFaults;
    Faults rightFollowerFaults;
    private Field2d m_field;
    private double maxSpeed;

    // ***********HAHAHAHA I MADE A COMMENT***************
    // Good Job!

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders

        _fxes.add(m_leftMotors);
        _fxes.add(m_leftFollower);
        _fxes.add(m_rightMotors);
        _fxes.add(m_rightFollower);
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
        m_rightFollower.follow(m_rightMotors);
        m_leftFollower.follow(m_leftMotors);
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

  static ArrayList<WPI_TalonFX> _fxes = new ArrayList<WPI_TalonFX>();
  private Field2d m_field;
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

        _fxes.add(m_leftMotors);
        _fxes.add(m_leftFollower);
        _fxes.add(m_rightMotors);
        _fxes.add(m_rightFollower);

  private double maxSpeed;

  public static ArrayList<String> Names;

  // ***********HAHAHAHA I MADE A COMMENT***************
  // Good Job!
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders

    _fxes.add(m_leftMotors);
    Names.add("LeftMaster ");
    _fxes.add(m_leftFollower);
    Names.add("LeftFollower ");
    _fxes.add(m_rightMotors);
    Names.add("RightMaster ");
    _fxes.add(m_rightFollower);
    Names.add("RightFollower ");
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_rightFollower.follow(m_rightMotors);
    m_leftFollower.follow(m_leftMotors);
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void customPeriodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(), m_leftEncoder.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse, -1 * m_rightEncoder.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse);
        m_field.setRobotPose(m_odometry.getPoseMeters());
        Pose2d translation = m_odometry.getPoseMeters();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());
        m_thetaEntry.setNumber(translation.getRotation().getDegrees());
        m_field.setRobotPose(m_odometry.getPoseMeters());

        //FAULTS
    }

    public String getMotorFaultsStr() {
        Faults prevLeftMasterFaults = leftMasterFaults;
        Faults prevRightMasterFaults = rightMasterFaults;
        Faults prevLeftFollowerFaults = leftFollowerFaults;
        Faults prevRightFollowerFaults = rightFollowerFaults;
        m_leftMotors.getFaults(leftMasterFaults);
        m_rightMotors.getFaults(rightMasterFaults);
        m_leftFollower.getFaults(leftFollowerFaults);
        m_rightFollower.getFaults(rightFollowerFaults);
        String leftMasterFaultsStr = convertFaultToStr(leftMasterFaults);
        String rightMasterFaultsStr = convertFaultToStr(rightMasterFaults);
        String leftFollowerFaultsStr = convertFaultToStr(leftFollowerFaults);
        String rightFollowerFaultsStr = convertFaultToStr(rightFollowerFaults);
        String returnStr = "";
        if (leftMasterFaultsStr != "") {
            returnStr += "{leftMaster: " + leftMasterFaultsStr + "}";
        }
        if (rightMasterFaultsStr != "") {
            returnStr += "{rightMaster: " + rightMasterFaultsStr + "}";
        }
        if (leftFollowerFaultsStr != "") {
            returnStr += "{leftFollower: " + leftFollowerFaultsStr + "}";
        }
        if (rightFollowerFaultsStr != "") {
            returnStr += "{rightFollower: " + rightFollowerFaultsStr + "}";
        }
        return returnStr;
    }

    String convertFaultToStr(Faults motorFaults) {
        String returnStr = "";
        if (motorFaults.APIError) {
            returnStr += "APIError, ";
        }
        if (motorFaults.ForwardLimitSwitch) {
            returnStr += "ForwardLimitSwitch, ";
        }
        if (motorFaults.ForwardSoftLimit) {
            returnStr += "ForwardSoftLimit, ";
        }
        if (motorFaults.HardwareESDReset) {
            returnStr += "HardwareESDReset, ";
        }
        if (motorFaults.HardwareFailure) {
            returnStr += "HardwareFailure, ";
        }
        if (motorFaults.RemoteLossOfSignal) {
            returnStr += "RemoteLossOfSignal, ";
        }
        if (motorFaults.ResetDuringEn) {
            returnStr += "ResetDuringEn, ";
        }
        if (motorFaults.ReverseLimitSwitch) {
            returnStr += "ReverseLimitSwitch, ";
        }
        if (motorFaults.ReverseSoftLimit) {
            returnStr += "ReverseSoftLimit, ";
        }
        if (motorFaults.SensorOutOfPhase) {
            returnStr += "SensorOutOfPhase, ";
        }
        if (motorFaults.SensorOverflow) {
            returnStr += "SensorOverflow, ";
        }
        if (motorFaults.SupplyOverV) {
            returnStr += "SupplyOverV, ";
        }
        if (motorFaults.SupplyUnstable) {
            returnStr += "SupplyUnstable, ";
        }
        if (motorFaults.UnderVoltage) {
            returnStr += "UnderVoltage, ";
        }
        return returnStr;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * 10, m_rightEncoder.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * -10);
    }

  public String getMotorFaultsStr() {
    return MotorFaultLogger.Logger(_fxes, Names);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }
}
