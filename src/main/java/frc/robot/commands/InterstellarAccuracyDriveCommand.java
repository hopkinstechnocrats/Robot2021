package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import lib.DelayedSequentialCommandGroupFactory;
import lib.TrajectoryCommandGenerator;
import lib.AutoCourses;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class InterstellarAccuracyDriveCommand extends CommandBase {
    public static Command getInstance(JoystickButton waitButton, DriveSubsystem driveSubsystem, PIDController left,
            PIDController right) {
        ArrayList<Trajectory> trajectories = AutoCourses.getIAC();
        Command[] trajectoryCommands = TrajectoryCommandGenerator
                .getTrajectoryCommand(trajectories, driveSubsystem, left, right).toArray(new Command[14]);
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.resetOdometry(trajectories.get(0).getInitialPose())),
                DelayedSequentialCommandGroupFactory.get(waitButton, trajectoryCommands));
    }
}