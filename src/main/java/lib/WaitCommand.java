package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class WaitCommand extends CommandBase {
  JoystickButton button;

  public WaitCommand(JoystickButton button) {
    // variable = expression;
    this.button = button;
  }

  public boolean isFinished() {
    return button.get(); 
  }
}