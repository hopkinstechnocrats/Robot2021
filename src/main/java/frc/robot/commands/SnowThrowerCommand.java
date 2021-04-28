package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PreLaunchSubsystem;
import lib.LoggableCommand;

public class SnowThrowerCommand extends ParallelCommandGroup implements LoggableCommand {
    private final LoggableCommand gscc;

    public SnowThrowerCommand(LauncherSubsystem m_launcherSubsystem, IntakeSubsystem m_intakeSubsystem, PreLaunchSubsystem m_PreLaunch, DriveSubsystem m_robotDrive) {
        gscc = new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "SnowThrower");
        addCommands(gscc,
        new ShootShootCommand(m_launcherSubsystem, m_PreLaunch));
    }

    public void logInit() {
        gscc.logInit();
    }

}