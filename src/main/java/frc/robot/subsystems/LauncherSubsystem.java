// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import lib.Loggable;
import lib.MotorFaultLogger;

public class LauncherSubsystem extends SubsystemBase implements Loggable {
    final WPI_TalonFX master;
    final WPI_TalonFX follower;
    final PIDController pidController;
    final SimpleMotorFeedforward feedforward;

    public LauncherSubsystem() {
        master = new WPI_TalonFX(LauncherConstants.Motor1CANID);
        follower = new WPI_TalonFX(LauncherConstants.Motor2CANID);
        follower.follow(master);
        follower.setInverted(true);
        pidController = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);
        feedforward = new SimpleMotorFeedforward(0.498, LauncherConstants.kV, LauncherConstants.kA);
        MotorFaultLogger.getInstance().add("LauncherMasterMotor", master);
        MotorFaultLogger.getInstance().add("LauncherFollowerMotor", follower);
    }

    public void logInit() {
        BadLog.createValue("Launcher/kP", "" + LauncherConstants.kP);
        BadLog.createValue("Launcher/kD", "" + LauncherConstants.kD);
        BadLog.createTopic("Launcher/Target Velocity", "rpm", pidController::getSetpoint);
        BadLog.createTopic("Launcher/Error", "rpm", pidController::getPositionError);
        BadLog.createTopic("Launcher/Launcher Velocity", "rpm", master::getSelectedSensorVelocity);
        BadLog.createValue("Launcher/kI", "" + LauncherConstants.kI);
    }


    // speed is in flywheel rpm
    public void spinLauncher(double speed) {
        double currentRPM = 10 * master.getSelectedSensorVelocity(0) / LauncherConstants.kEncoderUnitsPerRevolution;
        double motorVoltage = pidController.calculate(currentRPM, speed)+feedforward.calculate(speed);
        master.setVoltage(motorVoltage);
        follower.feed();
        SmartDashboard.putNumber("Launcher/Target Velocity", pidController.getSetpoint());
        SmartDashboard.putNumber("Launcher/Error", pidController.getPositionError()); // Actually velocity error
        SmartDashboard.putNumber("Launcher/Launcher Velocity", currentRPM);
    }

    public void stopLauncher() {
        master.setVoltage(0);
    }
}
