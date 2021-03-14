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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.SPI;


public class DriveSubsystem extends SubsystemBase {
  private class Temp implements Collection<WPI_TalonFX>{
    WPI_TalonFX[] items = new WPI_TalonFX[4];
    private Temp(){
      items[0] = m_leftMotors;
      items[1] = m_leftFollower;
      items[2] = m_rightMotors;
      items[3] = m_rightFollower;
    }
    public int size(){
      return items.length;
    }
    public boolean isEmpty(){
      return items.length == 0;
    }
    
    public boolean contains(java.lang.Object arg0){
      for(int i = 0; i < items.length; i++){
        if(arg0.equals(items[i])){
          return true;
        }
      }
      return false;
    }
    
    public java.util.Iterator<WPI_TalonFX> iterator(){
      return null;
    }
    
    public java.lang.Object[] toArray(){
      return items;
    }
    
    public WPI_TalonFX[] toArray(WPI_TalonFX[] arg0){
      return items;
    }

    public boolean add(WPI_TalonFX arg0){
      return false;
    }
    
    public  boolean remove(java.lang.Object arg0){
      return false;
    }
    
    public boolean containsAll(java.util.Collection<?> arg0){
      return false;
    }
    public <WPI_TalonFX> WPI_TalonFX[] toArray(WPI_TalonFX[] arg0){
      return null;
    }
    
    public boolean addAll(Collection<? extends WPI_TalonFX> arg0){
      return false;
    }
  
    public boolean removeAll(java.util.Collection<?> arg0){
      return false;
    }
    
    public boolean retainAll(java.util.Collection<?> arg0){
      return false;
    };
    
    public void clear(){

    }
    
    public boolean equals(java.lang.Object arg0){
      return false;
    }
    
    public  int hashCode(){
      return 0;
    }
  }
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

  Collection<WPI_TalonFX> _fxes = new Temp();

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
  //***********HAHAHAHA I MADE A COMMENT***************
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_rightFollower.follow(m_rightMotors);
    m_leftFollower.follow(m_leftMotors);
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);  

    _Orchestra = new Orchestra(_fxes);

    _Orchestra.loadMusic("Songs/MARIO.chrp");
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
