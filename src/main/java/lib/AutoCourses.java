package lib;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import static java.util.stream.Collectors.toList;

import badlog.lib.BadLog;

public class AutoCourses {
    public Trajectory barrelRacer;
    public Trajectory slalom;
    public Trajectory bounceCourse;

    public AutoCourses() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 10);
        BadLog.createValue("AutoConstants/MaxVoltage", "10");


        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);    



        //Barrel
        Pose2d startBarrel = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d finishBarrel = new Pose2d(-1.5, 0.3, new Rotation2d(Math.PI));
        List<Translation2d> waypointsBarrel = List.of(new Translation2d(2.286, 0.3048), new Translation2d(3.3528, -0.762),
        new Translation2d(2.286, -1.8288), new Translation2d(1.2192, -0.762),
        new Translation2d(2.286, 0.3048), new Translation2d(4.572, -0.3048),
        new Translation2d(5.6388, 0.762), new Translation2d(4.572, 1.8288),
        new Translation2d(3.5052, 0.762), new Translation2d(4.572, -0.3048),
        new Translation2d(6.096, -1.8288), new Translation2d(7.1628, -0.762),
        new Translation2d(6.096, 0.3048)); //Used Autonav Waypoint Calculator Sheet



        BadLog.createValue("Trajectory/Initial Desired Pose", ""+startBarrel);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finishBarrel);

        List<String> waypointBarrelStrings = waypointsBarrel.stream().map((o) -> o.toString()).collect(toList());
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointBarrelStrings));
        // An example trajectory to follow. All units in meters.
        barrelRacer = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBarrel,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsBarrel,
                // End 3 meters straight ahead of where we started, facing forward
                finishBarrel,
                // Pass config
                config);



        //Slalom
        Pose2d startSlalom = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d finishSlalom = new Pose2d(-1.5, 0.3, new Rotation2d(Math.PI));
        List<Translation2d> waypointsSlalom = List.of(new Translation2d(2.286, 0.3048), new Translation2d(3.3528, -0.762),
        new Translation2d(2.286, -1.8288), new Translation2d(1.2192, -0.762),
        new Translation2d(2.286, 0.3048), new Translation2d(4.572, -0.3048),
        new Translation2d(5.6388, 0.762), new Translation2d(4.572, 1.8288),
        new Translation2d(3.5052, 0.762), new Translation2d(4.572, -0.3048),
        new Translation2d(6.096, -1.8288), new Translation2d(7.1628, -0.762),
        new Translation2d(6.096, 0.3048)); //Used Autonav Waypoint Calculator Sheet



        BadLog.createValue("Trajectory/Initial Desired Pose", ""+startSlalom);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finishSlalom);

        List<String> waypointSlalomStrings = waypointsSlalom.stream().map((o) -> o.toString()).collect(toList());
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointSlalomStrings));
        // An example trajectory to follow. All units in meters.
        slalom = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startSlalom,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsSlalom,
                // End 3 meters straight ahead of where we started, facing forward
                finishSlalom,
                // Pass config
                config);

        
/*
        //Bounce
        Pose2d startBounce = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d finishBarrel = new Pose2d(-1.5, 0.3, new Rotation2d(Math.PI));
        List<Translation2d> waypointsBarrel = List.of(new Translation2d(2.286, 0.3048), new Translation2d(3.3528, -0.762),
        new Translation2d(2.286, -1.8288), new Translation2d(1.2192, -0.762),
        new Translation2d(2.286, 0.3048), new Translation2d(4.572, -0.3048),
        new Translation2d(5.6388, 0.762), new Translation2d(4.572, 1.8288),
        new Translation2d(3.5052, 0.762), new Translation2d(4.572, -0.3048),
        new Translation2d(6.096, -1.8288), new Translation2d(7.1628, -0.762),
        new Translation2d(6.096, 0.3048)); //Used Autonav Waypoint Calculator Sheet



        BadLog.createValue("Trajectory/Initial Desired Pose", ""+startBarrel);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finishBarrel);

        List<String> waypointSBarreltrings = waypointsBarrel.stream().map((o) -> o.toString()).collect(toList());
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointSBarreltrings));
        // An example trajectory to follow. All units in meters.
        barrelRacer = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBarrel,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsBarrel,
                // End 3 meters straight ahead of where we started, facing forward
                finishBarrel,
                // Pass config
                config);*/
    }
}