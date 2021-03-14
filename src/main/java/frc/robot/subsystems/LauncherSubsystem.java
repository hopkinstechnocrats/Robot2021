// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
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
  Orchestra _Orchestra;
  boolean singing;

  public LauncherSubsystem() {
    master = new WPI_TalonFX(LauncherConstants.Motor1CANID);
    follower = new WPI_TalonFX(LauncherConstants.Motor2CANID);
    follower.follow(master);
    ArrayList<TalonFX> _fxes = new ArrayList<>();
    _fxes.add(master);
    _fxes.add(follower);
    master.config_kP(0, LauncherConstants.kP, 10);
    master.config_kI(0, LauncherConstants.kI, 10);
    master.config_kD(0, LauncherConstants.kD, 10);
    _Orchestra = new Orchestra(_fxes);
    singing = false;

    _Orchestra.loadMusic("Songs/TTFAF.chrp");
  }

  public void initializeLog() {
      SlotConfiguration PIDConstants = new SlotConfiguration();
      master.getSlotConfigs(PIDConstants, 0, 50);
      BadLog.createValue("Launcher/kP", ""+PIDConstants.kP);
      BadLog.createValue("Launcher/kD", ""+PIDConstants.kD);
      BadLog.createTopic("Launcher/Target Velocity", "u/100ms", () -> master.getClosedLoopTarget(0));
      BadLog.createTopic("Launcher/Error", "u/100ms", () -> master.getClosedLoopError(0));
      BadLog.createTopic("Launcher/Launcher Velocity", "u/100ms", () -> master.getSelectedSensorVelocity(0));
      BadLog.createValue("Launcher/kI", ""+PIDConstants.kI);
  }

  public void Sing(){
    if(!_Orchestra.isPlaying()){
      _Orchestra.play();
    }else{
      _Orchestra.stop();
    }
  }


  public void spinLauncher(double speed) {
    master.set(ControlMode.Velocity, speed);
    follower.feed();
    SmartDashboard.putNumber("Launcher/Target Velocity", master.getClosedLoopTarget(0));
    SmartDashboard.putNumber("Launcher/Error", master.getClosedLoopError(0));
    SmartDashboard.putNumber("Launcher/Launcher Velocity", master.getSelectedSensorVelocity(0));
  }
}
