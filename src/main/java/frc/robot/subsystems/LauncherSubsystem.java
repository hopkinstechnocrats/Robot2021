// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.music.Orchestra;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lib.CustomDifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SPI;
import java.util.ArrayList;

public class LauncherSubsystem extends SubsystemBase {
  WPI_TalonFX master;
  WPI_TalonFX follower;
  PIDController pidController;

  public LauncherSubsystem() {
    master = new WPI_TalonFX(LauncherConstants.Motor1CANID);
    follower = new WPI_TalonFX(LauncherConstants.Motor2CANID);
    follower.follow(master);
    follower.setInverted(true);
    pidController = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);
  }

  public void initializeLog() {
      BadLog.createValue("Launcher/kP", ""+LauncherConstants.kP);
      BadLog.createValue("Launcher/kD", ""+LauncherConstants.kD);
      BadLog.createTopic("Launcher/Target Velocity", "rpm", () -> pidController.getSetpoint());
      BadLog.createTopic("Launcher/Error", "rpm", () -> pidController.getPositionError());
      BadLog.createTopic("Launcher/Launcher Velocity", "rpm", () -> master.getSelectedSensorPosition());
      BadLog.createValue("Launcher/kI", ""+LauncherConstants.kI);
  }


  // speed is in flywheel rpm
  public void spinLauncher(double speed) {
    double currentRPM = master.getSelectedSensorVelocity(0)/(600*LauncherConstants.kEncoderUnitsPerRevolution);
    double motorVoltage = pidController.calculate(currentRPM, speed);
    master.setVoltage(motorVoltage);
    follower.feed();
    SmartDashboard.putNumber("Launcher/Target Velocity", pidController.getSetpoint());
    SmartDashboard.putNumber("Launcher/Error", pidController.getPositionError()); // Actually velocity error
    SmartDashboard.putNumber("Launcher/Launcher Velocity", currentRPM);
  }
}
