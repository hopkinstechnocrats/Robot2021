package lib;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class TrajectoryCommandGenerator {

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, 10);

    public static final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    public static List<Command> getTrajectoryCommand(List<Trajectory> exampleTrajectory,
                                                     DriveSubsystem robotDrive, PIDController leftPIDController, PIDController rightPIDController) {
        ArrayList<Command> RamseteCommandList = new ArrayList<>();
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
        Command resetOdometryCommand = new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.get(0).getInitialPose()));
        List<Command> NewRamseteCommandList = new ArrayList<Command>();
        NewRamseteCommandList.add(resetOdometryCommand);
        NewRamseteCommandList.addAll(RamseteCommandList);

        return NewRamseteCommandList;
    }

    public static List<Command> getTrajectoryCommand(String trajectoryName, DriveSubsystem robotDrive, PIDController leftPIDController, PIDController rightPIDController) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("Paths/" + trajectoryName + ".json");
        String trajectoryInfo = "";
        try {
            trajectoryInfo = Files.readString(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open " + trajectoryName + " path", e.getStackTrace());
        }
        ObjectMapper mapper = new ObjectMapper();
        JsonNode headNode = null;
        try {
            headNode = mapper.readTree(trajectoryInfo);
        } catch (JsonProcessingException e) {
            DriverStation.reportError("Unable to process " + trajectoryName + " path", e.getStackTrace());
        }

        assert headNode != null;
        Iterator<JsonNode> pathsNodeArray = headNode.elements();
        List<TrajectoryWaypoints> pathsArray = new ArrayList<>();
        pathsNodeArray.forEachRemaining((JsonNode path) -> {
            TrajectoryWaypoints pathObject = new TrajectoryWaypoints();
            pathObject.start.x = path.get("start").get("x").asDouble();
            pathObject.start.y = path.get("start").get("y").asDouble();
            pathObject.start.theta = path.get("start").get("theta").asDouble();
            pathObject.finish.x = path.get("finish").get("x").asDouble();
            pathObject.finish.y = path.get("finish").get("y").asDouble();
            pathObject.finish.theta = path.get("finish").get("theta").asDouble();
            pathObject.reversed = path.get("reversed").asBoolean();
            List<double[]> waypointsList = new ArrayList<>();
            path.get("waypoints").elements().forEachRemaining((JsonNode waypoint) -> {
                double[] waypointArray = {waypoint.get(0).asDouble(), waypoint.get(1).asDouble()};
                waypointsList.add(waypointArray);
            });
            pathObject.waypoints = waypointsList.toArray(new double[waypointsList.size()][2]);
            pathsArray.add(pathObject);
        });

        List<Trajectory> trajectories = pathsArray.stream().map(
                (TrajectoryWaypoints waypointsInfo) -> {
                    Pose2d start = new Pose2d(Units.feetToMeters(waypointsInfo.start.x), Units.feetToMeters(waypointsInfo.start.y), Rotation2d.fromDegrees(waypointsInfo.start.theta));
                    Pose2d end = new Pose2d(Units.feetToMeters(waypointsInfo.finish.x), Units.feetToMeters(waypointsInfo.finish.y), Rotation2d.fromDegrees(waypointsInfo.finish.theta));
                    List<Translation2d> interiorWaypoints = Stream.of(waypointsInfo.waypoints).map((double[] waypoint) -> new Translation2d(Units.feetToMeters(waypoint[0]), Units.feetToMeters(waypoint[1]))).collect(Collectors.toList());
                    return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config.setReversed(waypointsInfo.reversed));
                }
        ).collect(Collectors.toList());
        return getTrajectoryCommand(trajectories, robotDrive, leftPIDController, rightPIDController);
    }

}
