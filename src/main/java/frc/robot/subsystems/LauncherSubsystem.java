// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        feedforward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);
        MotorFaultLogger.getInstance().add("LauncherMasterMotor", master);
        MotorFaultLogger.getInstance().add("LauncherFollowerMotor", follower);
    }

    public void logInit() {
        BadLog.createValue("Launcher/kP", "" + LauncherConstants.kP);
        BadLog.createValue("Launcher/kD", "" + LauncherConstants.kD);
        BadLog.createValue("Launcher/kI", "" + LauncherConstants.kI);
        BadLog.createValue("Launcher/kS", "" + LauncherConstants.kS);
        BadLog.createValue("Launcher/kV", "" + LauncherConstants.kV);
        BadLog.createValue("Launcher/kA", "" + LauncherConstants.kA);

        BadLog.createTopic("Launcher/Target Velocity", "rpm", pidController::getSetpoint, "join:Launcher/Launcher PID Loop");
        BadLog.createTopic("Launcher/Error", "rpm", pidController::getPositionError, "join:Launcher/Launcher PID Loop");
        BadLog.createTopicSubscriber("Launcher/Launcher Velocity", "rpm", DataInferMode.DEFAULT, "join:Launcher/Launcher PID Loop");
        BadLog.createTopicSubscriber("Launcher/Launcher Feedforward Voltage", "volts", DataInferMode.DEFAULT, "join:Launcher/Launcher PID Loop");
        BadLog.createTopicSubscriber("Launcher/Launcher Feedback Voltage", "volts", DataInferMode.DEFAULT, "join:Launcher/Launcher PID Loop");
        BadLog.createTopicSubscriber("Launcher/Total Voltage", "volts", DataInferMode.DEFAULT, "join:Launcher/Launcher PID Loop");

    }


    // speed is in flywheel rpm
    public void spinLauncher(double speed) {
        double currentRPM = master.getSelectedSensorVelocity(0) * LauncherConstants.kRawVelocityToRPM;
        BadLog.publish("Launcher/Launcher Velocity", currentRPM);
        double feedforwardVoltage = feedforward.calculate(speed);
        double feedbackVoltage = pidController.calculate(currentRPM, speed);
        double motorVoltage = feedbackVoltage + feedforwardVoltage;
        BadLog.publish("Launcher/Launcher Feedforward Voltage", feedforwardVoltage);
        BadLog.publish("Launcher/Launcher Feedback Voltage", feedbackVoltage);
        BadLog.publish("Launcher/Total Voltage", motorVoltage);

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
