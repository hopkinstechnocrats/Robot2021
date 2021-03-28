package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import lib.DelayedSequentialCommandGroupFactory;

public class InterstellarAccuracyDriveCommand extends CommandBase {
    public InterstellarAccuracyDriveCommand getInstance(JoystickButton waitButton) {
        
        return DelayedSequentialCommandGroupFactory.get(waitButton, 
        )
    }
}