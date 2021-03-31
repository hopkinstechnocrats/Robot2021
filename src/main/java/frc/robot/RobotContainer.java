// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import badlog.lib.BadLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoNavCommand;
import frc.robot.commands.GalacticSearchCommand;
import frc.robot.commands.SpinLauncherCommand;
import frc.robot.subsystems.*;
import lib.Loggable;
import lib.LoggableCommand;
import lib.MotorFaultLogger;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.List;

import static edu.wpi.first.networktables.NetworkTableInstance.*;
import static edu.wpi.first.wpilibj.XboxController.Button;

// import frc.robot.commands.InterstellarAccuracyDriveCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    static double speed = 1;
    // The robot's subsystems
    public final PreLaunchSubsystem m_PreLaunch;
    public final DriveSubsystem m_robotDrive;
    public final LauncherSubsystem m_launcherSubsystem;
    public final IntakeSubsystem m_intakeSubsystem;
    private final SendableChooser<LoggableCommand> autoChooser;
    private final PDPSubsystem m_PDPSubsystem;
    public BadLog log;
    public File logFile;
    public BufferedWriter logFileWriter;
    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private List<Loggable> loggables;
    private final NetworkTable metaLogTable = getDefault().getTable("metaLog");
    private final NetworkTableEntry TimeStamp = metaLogTable.getEntry("timeStamp");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        autoChooser = new SendableChooser<LoggableCommand>();
        m_robotDrive = new DriveSubsystem();
        m_launcherSubsystem = new LauncherSubsystem();
        m_intakeSubsystem = new IntakeSubsystem();
        m_PreLaunch = new PreLaunchSubsystem();
        m_PDPSubsystem = new PDPSubsystem();
        autoChooser.setDefaultOption("Barrel Racer", new AutoNavCommand(m_robotDrive, "BarrelRacer"));
        autoChooser.addOption("Bounce Course", new AutoNavCommand(m_robotDrive, "BouncePath"));
        autoChooser.addOption("Slalom", new AutoNavCommand(m_robotDrive, "Slalom"));
        autoChooser.addOption("GSCARed", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSAR"));
        autoChooser.addOption("GSCABlue", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSAB"));
        autoChooser.addOption("GSCBRed", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSBR"));
        autoChooser.addOption("GSCBBlue", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSBB"));


        SmartDashboard.putData(autoChooser);
        loggables.add(m_launcherSubsystem);
        loggables.add(m_robotDrive);
        loggables.add(MotorFaultLogger.getInstance());
        loggables.add(m_PDPSubsystem);
    }

    public void initializeLog() {
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
        String filepath = "/home/lvuser/logs" + timeStamp + ".bag";

        TimeStamp.setString(timeStamp);

        File file = new File(filepath);
        try {
            file.createNewFile();
            logFileWriter = new BufferedWriter(new FileWriter(file));
        } catch (IOException e) {
            DriverStation.reportError("File Creation error", e.getStackTrace());
        }

        log = BadLog.init(filepath);
        for (Loggable loggable : loggables) {
            loggable.logInit();
        }
    }

    public void logPeriodic() {
        for (Loggable loggable : loggables) {
            loggable.logPeriodic();
        }
    }

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
                .whileHeld(new RunCommand(() -> {
                    m_intakeSubsystem.spin(-1);
                    System.out.println("RUNNING INTAKE COMMAND");
                }));

        new JoystickButton(m_driverController, Button.kBumperRight.value)
                .whenPressed(new InstantCommand(() -> {
                    m_intakeSubsystem.toggle();
                    System.out.println("RUNNING INTAKE DEPLOY COMMAND");
                }));

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

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> m_robotDrive.tankDrivePercentOutput(m_driverController.getY(GenericHID.Hand.kLeft),
                        m_driverController.getY(GenericHID.Hand.kRight))
                        , m_robotDrive));

        // JoystickButton xButton = new JoystickButton(m_driverController, Button.kBumperLeft.value);
        // xButton.whenPressed(InterstellarAccuracyDriveCommand.getInstance(xButton, m_robotDrive, leftPIDController, rightPIDController));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public LoggableCommand getAutonomousCommand() {
        LoggableCommand command = autoChooser.getSelected();
        command.logInit();
        return autoChooser.getSelected();
    }
}
