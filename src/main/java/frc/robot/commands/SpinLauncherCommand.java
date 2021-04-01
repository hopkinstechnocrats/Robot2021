package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;
import lib.SpeedShot;

/**
 * An example command that uses an example subsystem.
 */
public class SpinLauncherCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LauncherSubsystem m_subsystem;
    private SpeedShot m_speedShot;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SpinLauncherCommand(LauncherSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_speedShot = new SpeedShot();
        SmartDashboard.putNumber("LauncherSpeed", 15);

    }

    public void initialize() {
        
    }

    public void execute() {
        // Get distance to target
        // double distanceToTarget = Limelight.getDistanceFromTarget();
        // Calculate desired speed of launcher wheel
        // double desiredLauncherSpeed = m_speedShot.getLauncherDesiredSpeed(distanceToTarget);
        double desiredLauncherSpeed = SmartDashboard.getNumber("LauncherSpeed", 15);
        System.out.println(desiredLauncherSpeed);
        // Send speed to launcher subsystem
        m_subsystem.spinLauncher(desiredLauncherSpeed);
    }
}
// object: m_subsystem
// method: spinLauncher
// parameters: 1 double (the speed we want )
// <object>.<method>(<parameters>)