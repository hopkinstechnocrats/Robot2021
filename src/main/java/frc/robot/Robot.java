// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.InterstellarAccuracyCommand;
import jdk.jfr.StackTrace;
import lib.LoggableCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    NetworkTableEntry maxVelocity;
    NetworkTableEntry maxAcceleration;
    NetworkTableEntry isEnabled;
    PowerDistributionPanel PDP;
    private LoggableCommand m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private boolean hasBeenEnabled;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        hasBeenEnabled = false;
        m_robotContainer = new RobotContainer();
        addPeriodic(m_robotContainer.m_robotDrive::customPeriodic, 0.01, 0.01);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        if (hasBeenEnabled) {
        //     DriverStation.reportError("exception to restart robot code", null);
        }
    }

    @Override
    public void disabledPeriodic() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        hasBeenEnabled = true;
        m_robotContainer.initializeLog();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.log.finishInitialization();
        // m_robotContainer.m_pixySubsystem.updatePath();
        // SmartDashboard.putString("GSCPathDetermination", m_robotContainer.m_pixySubsystem.getCurrentPath());


        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        m_robotContainer.log.updateTopics();
        m_robotContainer.log.log();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        hasBeenEnabled = true;
        m_robotContainer.initializeLog();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        
        new JoystickButton(m_robotContainer.m_driverController, Button.kX.value)
                .whenPressed(m_robotContainer.getAutonomousCommand());
//        m_robotContainer.iacCommand = new InterstellarAccuracyCommand(m_robotContainer.m_robotDrive, "IAC", m_robotContainer.iacButton);
//        m_robotContainer.iacCommand.logInit();
//        m_robotContainer.iacButton.whenPressed(m_robotContainer.iacCommand);
        m_robotContainer.log.finishInitialization();

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        m_robotContainer.log.updateTopics();
        m_robotContainer.log.log();
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

    }
}
