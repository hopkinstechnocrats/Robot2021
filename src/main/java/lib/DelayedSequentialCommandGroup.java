package lib;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DelayedSequentialCommandGroup extends SequentialCommandGroup {

    public DelayedSequentialCommandGroup(JoystickButton button, Command... commands) {
        ArrayList newCommands;
        for (int i = 0; i < commands.length; i++) {
            newCommands.add(new SequentialCommandGroup(commands[i], new WaitCommand(button)))
        }
        return new SequentialCommandGroup(newCommands);
    }

}