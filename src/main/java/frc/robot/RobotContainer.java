// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  SimpleMotorFeedforward feedforward;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
    POVButton upButton = new POVButton(m_driverController,0);
    POVButton rightButton = new POVButton(m_driverController,90);
    POVButton downButton = new POVButton(m_driverController,180);
    POVButton leftButton = new POVButton(m_driverController,270);

    upButton.whenPressed(new InstantCommand((()-> {m_robotDrive.setMaxSpeed(1);}), m_robotDrive));
    rightButton.whenPressed(new InstantCommand((()-> {m_robotDrive.setMaxSpeed(0.75);}), m_robotDrive));
    downButton.whenPressed(new InstantCommand((()-> {m_robotDrive.setMaxSpeed(0.6);}), m_robotDrive));
    leftButton.whenPressed(new InstantCommand((()-> {m_robotDrive.setMaxSpeed(0.4);}), m_robotDrive));
    m_robotDrive.setMaxSpeed(.6);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.tankDrivePercentOutput(
                    m_driverController.getY(GenericHID.Hand.kLeft),
                    m_driverController.getY(GenericHID.Hand.kRight)),
            m_robotDrive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.4));
    
    new JoystickButton(m_driverController, Button.kBumperLeft.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.8));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
            new Translation2d(2.286, 0.3048), 
            new Translation2d(3.3528, -0.762),
            new Translation2d(2.286, -1.8288),
            new Translation2d(1.2192, -0.762),
            new Translation2d(2.286, 0.3048),
            new Translation2d(6.096, -0.3048),
            new Translation2d(7.1628, 0.762),
            new Translation2d(6.096, 1.8288),
            new Translation2d(5.0292, 0.762),
            new Translation2d(6.096, -3.048)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0)),
            // Pass config
            config);

    feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);


    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    PIDController pidController = new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel);
    // return new RunCommand(() -> {
    //     double leftVolts = feedforward.calculate(0.5)+pidController.calculate(m_robotDrive.getWheelSpeeds().leftMetersPerSecond, 0.5);
    //     double rightVolts = feedforward.calculate(0.5)+pidController.calculate(m_robotDrive.getWheelSpeeds().rightMetersPerSecond, 0.5);
    //     m_robotDrive.tankDriveVolts(leftVolts, rightVolts);
    //     SmartDashboard.putNumber("Left Desired Speed", 0.5);
    //     SmartDashboard.putNumber("Right Desired Speed", 0.5);
    //     SmartDashboard.putNumber("Left Feedforward Voltage", leftVolts);
    //     SmartDashboard.putNumber("Right Feedforward Voltage", rightVolts);
    //     SmartDashboard.putNumber("Left Wheel Speed", m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
    //     SmartDashboard.putNumber("Right Wheel Speed", m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
    // }, m_robotDrive);
    return ramseteCommand;
}
}
