package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PreLaunchSubsystem;
import lib.LoggableCommand;

public class SnowThrowerCommand extends ParallelCommandGroup implements LoggableCommand {
    private final LoggableCommand gscc;

    public SnowThrowerCommand(IntakeSubsystem m_intakeSubsystem, DriveSubsystem m_robotDrive) {
        gscc = new GalacticSearchCommand(m_robotDrive, m_intakeSubsystem, "SnowPickerUpper");
        addCommands(gscc);
        //parallel(gscc, new )
    }

    public void logInit() {
        gscc.logInit();
    }

}