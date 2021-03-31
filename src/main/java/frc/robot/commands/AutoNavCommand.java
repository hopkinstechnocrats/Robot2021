package frc.robot.commands;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import lib.LoggableCommand;
import lib.TrajectoryCommandGenerator;

import java.util.List;

public class AutoNavCommand extends SequentialCommandGroup implements LoggableCommand {
    double startAutoTime;
    double finishAutoTime;

    public AutoNavCommand(DriveSubsystem subsystem, String pathName) {
        NetworkTable metaLogTable = NetworkTableInstance.getDefault().getTable("metaLog");
        NetworkTableEntry ElapsedAutoTime = metaLogTable.getEntry("elapsedAutoTime");
        List<Command> driveCommands = TrajectoryCommandGenerator.getTrajectoryCommand(pathName, subsystem, subsystem.leftPIDController, subsystem.rightPIDController);
        Command startClockCommand = new InstantCommand(() -> {
            this.startAutoTime = Timer.getFPGATimestamp();
        }, subsystem);
        Command stopClockCommand = new InstantCommand(() -> {
            this.finishAutoTime = Timer.getFPGATimestamp();
            double elapsedTime = this.finishAutoTime - this.startAutoTime;
            ElapsedAutoTime.setDouble(elapsedTime);
            System.out.println("Elapsed Time: " + elapsedTime);
            SmartDashboard.putNumber("Elapsed Auto Time", elapsedTime);
        }, subsystem);
        addCommands(startClockCommand, new SequentialCommandGroup(driveCommands.toArray(new Command[driveCommands.size()])), stopClockCommand);
    }

    public void logInit() {
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose X", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose X");
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose Y", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose Y");
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose Heading", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose Heading");
    }

    public void logPeriodic() {

    }
}
