package frc.robot.commands;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import lib.LoggableCommand;
import lib.TrajectoryCommandGenerator;
import lib.WaitCommand;

import java.util.List;

public class GalacticSearchCommand extends ParallelDeadlineGroup implements LoggableCommand {
    double startAutoTime;
    double finishAutoTime;
    final String pathName;

    public GalacticSearchCommand(DriveSubsystem subsystem, IntakeSubsystem intake, String pathName) {
        super(new RunCommand(() -> {
        }), new InstantCommand(() -> {
        }));
        this.pathName = pathName;
        NetworkTable metaLogTable = NetworkTableInstance.getDefault().getTable("metaLog");
        NetworkTableEntry TimeStamp = metaLogTable.getEntry("timeStamp");
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
        Command spinIntake = new RunCommand(() -> intake.spin(-0.68));
        setDeadline(new SequentialCommandGroup(startClockCommand, new SequentialCommandGroup(driveCommands.toArray(new Command[0])), stopClockCommand));
        addCommands(new SequentialCommandGroup(new InstantCommand(intake::toggle, intake), new edu.wpi.first.wpilibj2.command.WaitCommand(1.5
        ), spinIntake));
    }

    public void logInit() {
        BadLog.createValue("Path Name", pathName);
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose X", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose X");
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose Y", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose Y");
        BadLog.createTopicSubscriber("Drivetrain/Trajectory Pose Heading", "m", DataInferMode.DEFAULT, "join:Drivetrain/Robot Pose Heading");
    }

}