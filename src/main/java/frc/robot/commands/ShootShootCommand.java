package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PreLaunchSubsystem;

public class ShootShootCommand extends SequentialCommandGroup {
    public ShootShootCommand(LauncherSubsystem launcher, PreLaunchSubsystem prelaunch) {
        Command parallelGroup = new ParallelCommandGroup(new ConstantSpinLauncherCommand(launcher, Constants.LauncherConstants.blueZoneSpeed), new RunCommand(() -> prelaunch.spin(1)));
        addCommands(new WaitCommand(14), new ConstantSpinLauncherCommand(launcher, Constants.LauncherConstants.blueZoneSpeed), parallelGroup);
    }
}
