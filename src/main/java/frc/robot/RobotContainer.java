// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PreLaunchSubsystem;
// import frc.robot.commands.InterstellarAccuracyDriveCommand;
import frc.robot.commands.SpinLauncherCommand;
import lib.AutoCourses;
import lib.TrajectoryCommandGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import lib.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.security.Timestamp;
import java.sql.Driver;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

import javax.swing.TransferHandler;

import static java.util.stream.Collectors.toList;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
        public final PreLaunchSubsystem m_PreLaunch = new PreLaunchSubsystem();
    public final DriveSubsystem m_robotDrive;
    public final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
    public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public BadLog log;
    public PIDController leftPIDController;
    public PIDController rightPIDController;
    public Timer m_autoTimer;
    public double startAutoTime;
    public double finishAutoTime;
    public File logFile;
    public BufferedWriter logFileWriter;
    public SendableChooser<Command> AutoPoto = new SendableChooser<>();

    NetworkTableInstance metaLogTableBIG = NetworkTableInstance.getDefault();
    NetworkTable metaLogTable = metaLogTableBIG.getTable("metaLog");
    NetworkTableEntry ElapsedAutoTime = metaLogTable.getEntry("elapsedAutoTime");
    NetworkTableEntry TimeStamp = metaLogTable.getEntry("timeStamp");

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    SimpleMotorFeedforward feedforward;
    private PowerDistributionPanel PDP;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.startAutoTime = 0;
        // Configure the button bindings
        configureButtonBindings();
        m_robotDrive = new DriveSubsystem();
        initializeAutoLog();
        leftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
        rightPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
        new AutoCourses();
        AutoPoto.setDefaultOption("Barrel Racer", getAutonomousCommand(AutoCourses.getBarrelRacer()));
        AutoPoto.addOption("Bounce Course", getAutonomousCommand(AutoCourses.getBounce()));
        AutoPoto.addOption("Slalom", getAutonomousCommand(AutoCourses.getSlalom()));

        SmartDashboard.putData(AutoPoto);

        
        feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter);
        POVButton upButton = new POVButton(m_driverController, 0);
        POVButton rightButton = new POVButton(m_driverController, 90);
        POVButton downButton = new POVButton(m_driverController, 180);
        POVButton leftButton = new POVButton(m_driverController, 270);


        upButton.whenPressed(new InstantCommand((() -> {
            m_robotDrive.setMaxSpeed(1);
        }), m_robotDrive));
        rightButton.whenPressed(new InstantCommand((() -> {
            m_robotDrive.setMaxSpeed(0.75);
        }), m_robotDrive));
        downButton.whenPressed(new InstantCommand((() -> {
            m_robotDrive.setMaxSpeed(0.6);
        }), m_robotDrive));
        leftButton.whenPressed(new InstantCommand((() -> {
            m_robotDrive.setMaxSpeed(0.4);
        }), m_robotDrive));
        m_robotDrive.setMaxSpeed(.6);

        feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);
        
        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> m_robotDrive.tankDrivePercentOutput(m_driverController.getY(GenericHID.Hand.kLeft),
                        m_driverController.getY(GenericHID.Hand.kRight))
        , m_robotDrive));

    }

  public void initializeAutoLog() {
      String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
      String filepath = "/home/lvuser/logs"+timeStamp+".bag";

      TimeStamp.setString(timeStamp);

      File file = new File(filepath);      
      try {
          file.createNewFile();
          logFileWriter = new BufferedWriter(new FileWriter(file));
      } catch (IOException e) {
          DriverStation.reportError("File Creation error", e.getStackTrace());
      }
      
      log = BadLog.init(filepath);

      //DriveConstants
      BadLog.createValue("DriveConstants/TrackWidthMeters", ""+Constants.DriveConstants.kTrackwidthMeters);
      BadLog.createValue("DriveConstants/EncoderCPR", ""+Constants.DriveConstants.kEncoderCPR);
      BadLog.createValue("DriveConstants/GearRatio", ""+Constants.DriveConstants.kGearRatio);
      BadLog.createValue("DriveConstants/WheelDiameterMeters", ""+Constants.DriveConstants.kWheelDiameterMeters);
      BadLog.createValue("DriveConstants/EncoderDistancePerPulse", ""+Constants.DriveConstants.kEncoderDistancePerPulse);
      BadLog.createValue("DriveConstants/ksVolts", ""+Constants.DriveConstants.ksVolts);
      BadLog.createValue("DriveConstants/kvVoltSecondsPerMeter", ""+Constants.DriveConstants.kvVoltSecondsPerMeter);
      BadLog.createValue("DriveConstants/kaVoltSecondsSquaredPerMeter", ""+Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
      BadLog.createValue("DriveConstants/kPDriveVel", ""+Constants.DriveConstants.kPDriveVel);
      BadLog.createValue("DriveConstants/kDDriveVel", ""+Constants.DriveConstants.kDDriveVel);

      //AutoConstants
      BadLog.createValue("AutoConstants/MaxSpeedMetersPerSecond", ""+Constants.AutoConstants.kMaxSpeedMetersPerSecond);
      BadLog.createValue("AutoConstants/MaxAccelerationMetersPerSecondSquared", ""+Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      BadLog.createValue("AutoConstants/RamseteB", ""+Constants.AutoConstants.kRamseteB);
      BadLog.createValue("AutoConstants/RamseteZeta", ""+Constants.AutoConstants.kRamseteZeta);

      BadLog.createTopic("FPGA Time", "s", () -> Timer.getFPGATimestamp(), "xaxis");
      BadLog.createTopic("Drivetrain/Odometry Pose X", "m", () -> m_robotDrive.getPose().getX(), "join:Drivetrain/Robot Pose X");
      BadLog.createTopic("Drivetrain/Odometry Pose Y", "m", () -> m_robotDrive.getPose().getY(), "join:Drivetrain/Robot Pose Y");
      BadLog.createTopic("Drivetrain/Odometry Pose Heading", "degrees", () -> m_robotDrive.getPose().getRotation().getDegrees(), "join:Drivetrain/Robot Pose Heading");
      BadLog.createTopic("Drivetrain/Left Wheel Measured Speed", "m/s", () -> m_robotDrive.getWheelSpeeds().leftMetersPerSecond, "join:Drivetrain/LeftWheelSpeed");
      BadLog.createTopic("Drivetrain/Right Wheel Measured Speed", "m/s", () -> m_robotDrive.getWheelSpeeds().rightMetersPerSecond, "join:Drivetrain/RightWheelSpeed");
      BadLog.createTopic("Drivetrain/Left Wheel Setpoint", "m/s" ,() -> leftPIDController.getSetpoint(), "join:Drivetrain/LeftWheelSpeed");
      BadLog.createTopic("Drivetrain/Right Wheel Setpoint", "m/s", () -> rightPIDController.getSetpoint(), "join:Drivetrain/RightWheelSpeed");
      BadLog.createTopic("Battery Voltage", "V", () -> RobotController.getBatteryVoltage());

      //Gyro Data
      BadLog.createTopic("NavX/WorldLinearAccelX", "m/s^2", () -> (double) m_robotDrive.navX.getWorldLinearAccelX());
      BadLog.createTopic("NavX/WorldLinearAccelY", "m/s^2", () -> (double) m_robotDrive.navX.getWorldLinearAccelX());
      BadLog.createTopic("NavX/WorldLinearAccelZ", "m/s^2", () -> (double) m_robotDrive.navX.getWorldLinearAccelX());
      BadLog.createTopic("NavX/RawAccelX", "m/s^2", () -> (double) m_robotDrive.navX.getRawAccelX());
      BadLog.createTopic("NavX/RawAccelY", "m/s^2", () -> (double) m_robotDrive.navX.getRawAccelX());
      BadLog.createTopic("NavX/RawAccelZ", "m/s^2", () -> (double) m_robotDrive.navX.getRawAccelX());

      BadLog.createTopicStr("Drivetrain/Motor Faults", BadLog.UNITLESS, m_robotDrive::getMotorFaultsStr, "log");
      PDP = new PowerDistributionPanel();

      for(int i = 0; i <=15; i++){
        final int j = i;
        BadLog.createTopic("PDP/PDP"+j+" Current", "Amperes", () -> PDP.getCurrent(j));
      }
      BadLog.createTopic("PDP/PDP Temp", "Celsius", () -> PDP.getTemperature());
      BadLog.createTopic("PDP/PDP Total Current", "Amperes", () -> PDP.getTotalCurrent());
      BadLog.createTopic("PDP/PDP Total Energy", "Joules", () -> PDP.getTotalEnergy());
      BadLog.createTopic("PDP/PDP Total Power", "Watts", () -> PDP.getTotalPower());
      BadLog.createTopic("PDP/PDP Input Voltage", "Volts", () -> PDP.getVoltage());
  }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    static double speed = 1;
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kBumperRight.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.4));

        new JoystickButton(m_driverController, Button.kBumperLeft.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.8));

        
        new JoystickButton(m_driverController, Button.kA.value)
                .whileHeld(new RunCommand(() -> m_launcherSubsystem.spinLauncher(LauncherConstants.speed), m_launcherSubsystem));

        new JoystickButton(m_driverController, Button.kX.value)
                .whileHeld(new SpinLauncherCommand(m_launcherSubsystem));

        // new JoystickButton(m_driverController, Button.kY.value)
        //         .whenPressed(new RunCommand(() -> m_robotDrive._Orchestra.play(), m_robotDrive));
    
        m_launcherSubsystem.setDefaultCommand(new RunCommand(() -> m_launcherSubsystem.spinLauncher(0), m_launcherSubsystem));

        new JoystickButton(m_driverController, Button.kY.value)
                .whileHeld(() -> m_PreLaunch.spin(-1), m_PreLaunch);

        m_PreLaunch.setDefaultCommand(new RunCommand(() -> m_PreLaunch.spin(0), m_PreLaunch));

        m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.spin(0), m_intakeSubsystem));

        new JoystickButton(m_driverController, Button.kB.value)
            .whileHeld(new RunCommand(() -> {m_intakeSubsystem.spin(-1); System.out.println("RUNNING INTAKE COMMAND");}));

        new JoystickButton(m_driverController, Button.kBumperRight.value)
            .whenPressed(new InstantCommand(() -> {m_intakeSubsystem.toggle(); System.out.println("RUNNING INTAKE DEPLOY COMMAND");}));
            
        // JoystickButton xButton = new JoystickButton(m_driverController, Button.kBumperLeft.value);
        // xButton.whenPressed(InterstellarAccuracyDriveCommand.getInstance(xButton, m_robotDrive, leftPIDController, rightPIDController));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command WhatAuto() {
        return AutoPoto.getSelected();
    }

    public Command getAutonomousCommand(ArrayList<Trajectory> Course) {
        
        ArrayList<Trajectory> exampleTrajectory = new ArrayList<Trajectory>();
        
        exampleTrajectory = Course;
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose X", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose X");
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose Y", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose Y");
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose Heading", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose Heading");

        Command startClockCommand = new InstantCommand(() -> {
            this.startAutoTime = Timer.getFPGATimestamp();
        }, m_robotDrive);
        Command stopClockCommand = new InstantCommand(() -> {
            this.finishAutoTime = Timer.getFPGATimestamp();
            double elapsedTime = this.finishAutoTime - this.startAutoTime;

            ElapsedAutoTime.setDouble(elapsedTime);

            System.out.println("Elapsed Time: " + elapsedTime);
            SmartDashboard.putNumber("Elapsed Auto Time", elapsedTime);
            try {
                logFileWriter.append("{\"elapsed_auto_time\":"+elapsedTime+"}");
            } catch (IOException e) {
                DriverStation.reportError("Cannot write auto time", e.getStackTrace());
            }
        }, m_robotDrive);


        ArrayList<RamseteCommand> RamseteCommandList;
        
        RamseteCommandList = TrajectoryCommandGenerator.getTrajectoryCommand(exampleTrajectory, m_robotDrive, leftPIDController, rightPIDController);
/*
        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds, leftPIDController, rightPIDController,
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);*/

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.get(0).getInitialPose());

        // Run path following command, then stop at the end.
        // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

        PIDController pidController = new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel);
        // return new RunCommand(() -> {
        // double leftVolts =
        // feedforward.calculate(0.5)+pidController.calculate(m_robotDrive.getWheelSpeeds().leftMetersPerSecond,
        // 0.5);
        // double rightVolts =
        // feedforward.calculate(0.5)+pidController.calculate(m_robotDrive.getWheelSpeeds().rightMetersPerSecond,
        // 0.5);
        // m_robotDrive.tankDriveVolts(leftVolts, rightVolts);
        // SmartDashboard.putNumber("Left Desired Speed", 0.5);
        // SmartDashboard.putNumber("Right Desired Speed", 0.5);
        // SmartDashboard.putNumber("Left Feedforward Voltage", leftVolts);
        // SmartDashboard.putNumber("Right Feedforward Voltage", rightVolts);
        // SmartDashboard.putNumber("Left Wheel Speed",
        // m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
        // SmartDashboard.putNumber("Right Wheel Speed",
        // m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
        // }, m_robotDrive);

        SequentialCommandGroup ramseteCommandGroup = new SequentialCommandGroup();
        for(RamseteCommand b:RamseteCommandList){
            ramseteCommandGroup.addCommands(b);
        }

        return new SequentialCommandGroup(startClockCommand, ramseteCommandGroup, stopClockCommand);
    }
}
