package lib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import lib.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DelayedSequentialCommandGroupFactory extends SequentialCommandGroup {

    public static SequentialCommandGroup get(JoystickButton button, Command... commands) {
        Command[] newCommands = new Command[commands.length];
        for (int i = 0; i < commands.length; i++) {
            newCommands[i] = (new SequentialCommandGroup(commands[i], new WaitCommand(button)));
        }
        return new SequentialCommandGroup(newCommands);
    }

}