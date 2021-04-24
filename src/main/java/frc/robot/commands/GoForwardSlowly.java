package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class GoForwardSlowly extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    double speed;
    private final DriveSubsystem m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public GoForwardSlowly(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        speed = 1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    public void initialize() {
        
    }

    public void execute() {
        m_subsystem.GoForwards(speed);
    }
}
