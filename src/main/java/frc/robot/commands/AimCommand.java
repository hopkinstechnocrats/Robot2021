
package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import lib.Limelight;

/**
 * An example command that uses an example subsystem.
 */
public class AimCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    private PIDController calculations;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        calculations = new PIDController(1, 0, 0);
    }

    @Override
    public void execute() {
        double angle = Limelight.getAngleFromTarget();
        double setPoint = calculations.calculate(angle, 0);
        m_subsystem.tankDriveVelocity(-setPoint, setPoint);
    }
}