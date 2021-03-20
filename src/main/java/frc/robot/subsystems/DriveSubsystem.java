// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lib.CustomDifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.IllegalFormatCodePointException;

import javax.management.openmbean.ArrayType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  public Orchestra _Orchestra;
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

  // The gyro sensor
  public AHRS navX = new AHRS(SPI.Port.kMXP);
  public final Gyro m_gyro = navX;

  private Field2d m_field;
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  NetworkTableEntry m_thetaEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("theta");

  private double maxSpeed;
  Faults leftMasterFaults;
  Faults rightMasterFaults;
  Faults leftFollowerFaults;
  Faults rightFollowerFaults;

  // ***********HAHAHAHA I MADE A COMMENT***************
  // Good Job!
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_rightFollower.follow(m_rightMotors);
    m_leftFollower.follow(m_leftMotors);
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    ArrayList<TalonFX> _fxes = new ArrayList<>();

    _fxes.add(m_leftMotors);
    _fxes.add(m_leftFollower);
    _fxes.add(m_rightMotors);
    _fxes.add(m_rightFollower);

    _Orchestra = new Orchestra(_fxes);

    _Orchestra.loadMusic("Songs/MARIO.chrp");
    leftMasterFaults = new Faults();
    rightMasterFaults = new Faults();
    leftFollowerFaults = new Faults();
    rightFollowerFaults = new Faults();

  }
  
  public void setMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
  }

  public void customPeriodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse, -1*m_rightEncoder.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse);
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
      returnStr += "{leftMaster: "+leftMasterFaultsStr+"}";
    }
    if (rightMasterFaultsStr != "") {
      returnStr += "{rightMaster: "+rightMasterFaultsStr+"}";
    }
    if (leftFollowerFaultsStr != "") {
      returnStr += "{leftFollower: "+leftFollowerFaultsStr+"}";
    }
    if (rightFollowerFaultsStr != "") {
      returnStr += "{rightFollower: "+rightFollowerFaultsStr+"}";
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
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getSelectedSensorVelocity()*DriveConstants.kEncoderDistancePerPulse*10, m_rightEncoder.getSelectedSensorVelocity()*DriveConstants.kEncoderDistancePerPulse*-10);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDrivePercentOutput(double left, double right) {
    m_drive.tankDrive(-1*maxSpeed*left, -1*maxSpeed*right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public void tankDriveVelocity(double leftVel, double rightVel) {
    m_leftMotors.set(ControlMode.Velocity, (1/(10*DriveConstants.kEncoderDistancePerPulse))*leftVel);
    System.out.println( (1/(10*DriveConstants.kEncoderDistancePerPulse))*leftVel);
    m_leftFollower.feed();
    m_rightMotors.set(ControlMode.Velocity, -(1/(10*DriveConstants.kEncoderDistancePerPulse))*rightVel);
    System.out.println((1/(10*DriveConstants.kEncoderDistancePerPulse))*rightVel);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setSelectedSensorPosition(0);
    m_rightEncoder.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse + -1*m_rightEncoder.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse) / 2.0;
  }


  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
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
