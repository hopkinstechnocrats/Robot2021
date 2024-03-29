// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import badlog.lib.BadLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import lib.Loggable;
import lib.LoggableCommand;
import lib.MotorFaultLogger;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
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
@SuppressWarnings("SpellCheckingInspection")
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
    public final HoodSubsystem m_hoodSubsystem;
//     public final PixySubsystem m_pixySubsystem;
    private final SendableChooser<LoggableCommand> autoChooser;
    public BadLog log;
    public File logFile;
    public BufferedWriter logFileWriter;
    // The driver's controller
    final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    private final List<Loggable> loggables;
    private final NetworkTable metaLogTable = getDefault().getTable("metaLog");
    private final NetworkTableEntry TimeStamp = metaLogTable.getEntry("timeStamp");
    InterstellarAccuracyCommand iacCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        loggables = new ArrayList<>();
        autoChooser = new SendableChooser<>();
        m_robotDrive = new DriveSubsystem();
        m_launcherSubsystem = new LauncherSubsystem();
        m_intakeSubsystem = new IntakeSubsystem();
        m_PreLaunch = new PreLaunchSubsystem();
        m_hoodSubsystem = new HoodSubsystem();
        // m_pixySubsystem = new PixySubsystem();
        PDPSubsystem m_PDPSubsystem = new PDPSubsystem();
        // autoChooser.setDefaultOption("Barrel Racer", new AutoNavCommand(m_robotDrive, "BarrelRacer"));
        // autoChooser.addOption("Bounce Course", new AutoNavCommand(m_robotDrive, "BouncePath"));
        // autoChooser.addOption("Test", new AutoNavCommand(m_robotDrive, "Test"));
        // autoChooser.addOption("Slalom", new AutoNavCommand(m_robotDrive, "Slalom"));
        // autoChooser.addOption("GSCARed", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSAR"));
        //  autoChooser.addOption("SnowThrower", new SnowThrowerCommand(m_launcherSubsystem, m_intakeSubsystem, m_PreLaunch, m_robotDrive));
        // autoChooser.addOption("GSCABlue", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSAB"));
        // autoChooser.addOption("GSCBRed", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSBR"));
        // autoChooser.addOption("GSCBBlue", new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "GSBB"));
        SmartDashboard.putData(autoChooser);
        loggables.add(m_launcherSubsystem);
        loggables.add(m_robotDrive);
        loggables.add(MotorFaultLogger.getInstance());
        loggables.add(m_PDPSubsystem);
        configureButtonBindings();

    }

    public void initializeLog() {
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
        String filepath = "/home/lvuser/logs" + timeStamp + ".bag";

        TimeStamp.setString(timeStamp);

        File file = new File(filepath);
        try {
            //noinspection ResultOfMethodCallIgnored
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

    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kBumperRight.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.4));

        new JoystickButton(m_driverController, Button.kBumperLeft.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.8));


        // new JoystickButton(m_operatorController, Button.kA.value)
        //         .whileHeld(new RunCommand(() -> m_launcherSubsystem.spinLauncher(LauncherConstants.speed), m_launcherSubsystem));

        // new JoystickButton(m_operatorController, Button.kX.value)
        //         .whileHeld(new SpinLauncherCommand(m_launcherSubsystem));


        // new JoystickButton(m_operatorController, Button.kY.value)
        //         .whenPressed(new RunCommand(() -> m_robotDrive._Orchestra.play(), m_robotDrive));

        m_launcherSubsystem.setDefaultCommand(new RunCommand(m_launcherSubsystem::stopLauncher, m_launcherSubsystem));

        new JoystickButton(m_operatorController, Button.kBack.value)
                .whileHeld(() -> m_PreLaunch.spin(-1), m_PreLaunch);

        new JoystickButton(m_driverController, Button.kStart.value)
                .whenPressed(m_robotDrive::toggleDirection);

        m_PreLaunch.setDefaultCommand(new RunCommand(() -> m_PreLaunch.spin(0), m_PreLaunch));

        m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.spin(0), m_intakeSubsystem));

        new JoystickButton(m_operatorController, Button.kStickRight.value)
                .whileHeld(new RunCommand(() -> {
                    m_intakeSubsystem.spin(-1);
                    System.out.println("RUNNING INTAKE COMMAND");
                }));

        new JoystickButton(m_operatorController, Button.kStickLeft.value)
                .whenPressed(new InstantCommand(() -> {
                    m_intakeSubsystem.toggle();
                    System.out.println("RUNNING INTAKE DEPLOY COMMAND");
                }));
        new JoystickButton(m_operatorController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> {
                    m_hoodSubsystem.toggle();
                    System.out.println("RUNNING HOOD DEPLOY COMMAND");
                }));

        new JoystickButton(m_operatorController, Button.kA.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.greenZoneSpeed));
        new JoystickButton(m_operatorController, Button.kY.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.yellowZoneSpeed));
        new JoystickButton(m_operatorController, Button.kX.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.blueZoneSpeed));
        new JoystickButton(m_operatorController, Button.kB.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.redZoneSpeed));
        new JoystickButton(m_driverController, Button.kY.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.blueSpecialZoneSpeed));
        new JoystickButton(m_driverController, Button.kB.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.blueExtraSpecialZoneSpeed));

        //new JoystickButton(m_operatorController, Button.kBumperRight.value)
        //        .whileHeld(new GoForwardSlowly(m_robotDrive));

        /*
        new JoystickButton(m_operatorController, Button.kBumperRight.value)
                .whileHeld(new ConstantSpinLauncherCommand(m_launcherSubsystem, LauncherConstants.snowThrowerSpeed));
                */

        POVButton upButton = new POVButton(m_driverController, 0);
        POVButton rightButton = new POVButton(m_driverController, 90);
        POVButton downButton = new POVButton(m_driverController, 180);
        POVButton leftButton = new POVButton(m_driverController, 270);

        
        upButton.whenPressed(new InstantCommand((() -> m_robotDrive.setMaxSpeed(1)), m_robotDrive));
        rightButton.whenPressed(new InstantCommand((() -> m_robotDrive.setMaxSpeed(0.75)), m_robotDrive));
        downButton.whenPressed(new InstantCommand((() -> m_robotDrive.setMaxSpeed(0.6)), m_robotDrive));
        leftButton.whenPressed(new InstantCommand((() -> m_robotDrive.setMaxSpeed(0.4)), m_robotDrive));
        m_robotDrive.setMaxSpeed(.6);

        POVButton operatorUpButton = new POVButton(m_operatorController, 0);
        POVButton operatorRightButton = new POVButton(m_operatorController, 90);
        POVButton operatorLeftButton = new POVButton(m_operatorController, 270);

        /*
        operatorUpButton.whenPressed(new SequentialCommandGroup(new InstantCommand((() -> m_robotDrive.zeroHeading()), m_robotDrive), 
                                     new SnowThrowerCommand(m_intakeSubsystem, m_robotDrive)));
        
        operatorLeftButton.whenPressed(new InstantCommand((() -> m_robotDrive.setDefaultCommand(
                new RunCommand(() -> m_robotDrive.tankDrivePercentOutput(.2*m_operatorController.getX(GenericHID.Hand.kLeft),
                -.2*m_operatorController.getX(GenericHID.Hand.kLeft)), m_robotDrive)))));
        
        operatorRightButton.whenPressed(new InstantCommand((() -> m_robotDrive.setDefaultCommand(
                new RunCommand(() -> m_robotDrive.tankDrivePercentOutput(m_driverController.getY(GenericHID.Hand.kLeft),
                m_driverController.getY(GenericHID.Hand.kRight)), m_robotDrive)))));
        */
        // m_pixySubsystem.setDefaultCommand(new RunCommand(() -> {

        //     Pixy2CCC.Block largestblock = m_pixySubsystem.getBiggestBlock();
        //     if (largestblock == null) {
        //         System.out.println("Found no blocks");
        //     } else {
        //         System.out.println("Block X: "+largestblock.getX()+", Block Y: "+largestblock.getY());

        //     }
        // },m_pixySubsystem));

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
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
        LoggableCommand command = new AutoNavCommand(m_robotDrive, "BarrelWeaver");
        command.logInit();
        return command;
    }
}
