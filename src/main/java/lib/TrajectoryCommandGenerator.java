package lib;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.namespace.QName;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import lib.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class TrajectoryCommandGenerator {

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                            DriveConstants.kaVoltSecondsSquaredPerMeter),
                            DriveConstants.kDriveKinematics, 10);

    public static final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

    public static ArrayList<RamseteCommand> getTrajectoryCommand(List<Trajectory> exampleTrajectory,
            DriveSubsystem robotDrive, PIDController leftPIDController, PIDController rightPIDController) {
        ArrayList<RamseteCommand> RamseteCommandList = new ArrayList<RamseteCommand>();
        for (Trajectory trajectory : exampleTrajectory) {
            RamseteCommand ramseteCommand = new RamseteCommand(trajectory, robotDrive::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                            DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics, robotDrive::getWheelSpeeds, leftPIDController,
                    rightPIDController,
                    // RamseteCommand passes volts to the callback
                    robotDrive::tankDriveVolts, robotDrive);

            RamseteCommandList.add(ramseteCommand);
        }

        return RamseteCommandList;
    }

    public static List<RamseteCommand> getTrajectoryCommand(String trajectoryName, DriveSubsystem robotDrive, PIDController leftPIDController, PIDController rightPIDController) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("Paths/"+trajectoryName+".json");
        String trajectoryInfo = "";
        try {
            trajectoryInfo = Files.readString(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open "+trajectoryName+" path", e.getStackTrace());
        }
        ObjectMapper mapper = new ObjectMapper();
        TrajectoryWaypoints[] waypointsArray = new TrajectoryWaypoints[1];
        try {
            waypointsArray = mapper.readValue(trajectoryInfo, TrajectoryWaypoints[].class);
        } catch (JsonProcessingException e) {
            DriverStation.reportError("Unable to process "+trajectoryName+" path", e.getStackTrace());
        }
        
        List<Trajectory> trajectories = Stream.of(waypointsArray).map(
            (TrajectoryWaypoints waypointsInfo) -> {
                Pose2d start = new Pose2d(waypointsInfo.start.x, waypointsInfo.start.y, Rotation2d.fromDegrees(waypointsInfo.start.theta));
                Pose2d end = new Pose2d(waypointsInfo.finish.x, waypointsInfo.finish.y, Rotation2d.fromDegrees(waypointsInfo.finish.theta));
                List<Translation2d> interiorWaypoints = Stream.of(waypointsInfo.waypoints).map((double[] waypoint) -> new Translation2d(Units.feetToMeters(waypoint[0]), Units.feetToMeters(waypoint[1]))).collect(Collectors.toList());
                return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config.setReversed(waypointsInfo.reversed));
            }
        ).collect(Collectors.toList());
        return getTrajectoryCommand(trajectories, robotDrive, leftPIDController, rightPIDController);
    }

}
