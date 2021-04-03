package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;
import lib.SpeedShot;

/**
 * An example command that uses an example subsystem.
 */
public class ConstantSpinLauncherCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LauncherSubsystem m_subsystem;
    private final double speed;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ConstantSpinLauncherCommand(LauncherSubsystem subsystem, double speed) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        SpeedShot m_speedShot = new SpeedShot();
        SmartDashboard.putNumber("LauncherSpeed", 15);
        this.speed = speed;

    }

    public void initialize() {

    }

    public void execute() {
        // Get distance to target
        // double distanceToTarget = Limelight.getDistanceFromTarget();
        // Calculate desired speed of launcher wheel
        // double desiredLauncherSpeed = m_speedShot.getLauncherDesiredSpeed(distanceToTarget);
        double desiredLauncherSpeed = SmartDashboard.getNumber("LauncherSpeed", 15);
        System.out.println(speed);
        // Send speed to launcher subsystem
        m_subsystem.spinLauncher(speed);
    }
}
// object: m_subsystem
// method: spinLauncher
// parameters: 1 double (the speed we want )
