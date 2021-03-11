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

import java.util.ArrayList;

import badlog.lib.BadLog;

public class AutoCourses {
    public Trajectory barrelRacer;
    public Trajectory slalom;
    public Trajectory bounceCourse1;
    public Trajectory bounceCourse2;
    public Trajectory bounceCourse3;
    public Trajectory bounceCourse4;
    public Trajectory bounceCourse;
    private final Pose2d startSlalom = new Pose2d(0, -1.524, new Rotation2d(0));
    private final Pose2d finishSlalom = new Pose2d(-1, 0.3, new Rotation2d(Math.PI));
    private final Pose2d finishBarrel = new Pose2d(-1.5, 0.3, new Rotation2d(Math.PI));
    private final Pose2d startBarrel = new Pose2d(0, 0, new Rotation2d(0));
    private final List<String> waypointBarrelStrings;
    private final List<String> waypointSlalomStrings;
    private final List<String> waypointBounce2Strings;
    private final List<String> waypointBounce1Strings;
    private final Pose2d finishBounce1 = new Pose2d(0.762, 1.676, new Rotation2d(Math.PI));
    private final Pose2d startBounce1 = new Pose2d(0, 0, new Rotation2d(0));
    private final Pose2d finishBounce2 = new Pose2d(3.048, 1.676, new Rotation2d(Math.PI));
    private final Pose2d startBounce2 = new Pose2d(0.762, 1.676, new Rotation2d(-90));
    private final Pose2d finishBounce3 = new Pose2d(5.334, 1.676, new Rotation2d(Math.PI));
    private final Pose2d startBounce3 = new Pose2d(3.048, 1.676, new Rotation2d(90));
    private final Pose2d finishBounce4 = new Pose2d(7.096, 0.305, new Rotation2d(Math.PI));
    private final Pose2d startBounce4 = new Pose2d(5.334, 1.676, new Rotation2d(-90));
    private final List<String> waypointBounce4Strings;
    private final List<String> waypointBounce3Strings;

    public AutoCourses() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        final var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 10);
        BadLog.createValue("AutoConstants/MaxVoltage", "10");


        // Create config for trajectory
        final TrajectoryConfig forwardConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);   
                
        final TrajectoryConfig reverseConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);  



        final List<Translation2d> waypointsBarrel = List.of(new Translation2d(2.286, 0.3048), new Translation2d(3.3528, -0.762),
        new Translation2d(2.286, -1.8288), new Translation2d(1.2192, -0.762),
        new Translation2d(2.286, 0.3048), new Translation2d(4.572, -0.3048),
        new Translation2d(5.6388, 0.762), new Translation2d(4.572, 1.8288),
        new Translation2d(3.5052, 0.762), new Translation2d(4.572, -0.3048),
        new Translation2d(6.096, -1.8288), new Translation2d(7.1628, -0.762),
        new Translation2d(6.096, 0.3048)); //Used Autonav Waypoint Calculator Sheet


        waypointBarrelStrings = waypointsBarrel.stream().map((o) -> o.toString()).collect(toList());
        // An example trajectory to follow. All units in meters.
        barrelRacer = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBarrel,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsBarrel,
                // End 3 meters straight ahead of where we started, facing forward
                finishBarrel,
                // Pass config
                forwardConfig);



        final List<Translation2d> waypointsSlalom = List.of(new Translation2d(0.762, -0.762), new Translation2d(1.524, 0),
        new Translation2d(4.572, 0), new Translation2d(5.334, -0.762),
        new Translation2d(6.096, -1.524), new Translation2d(6.858, -0.762),
        new Translation2d(6.096, 0), new Translation2d(5.234, -0.762),
        new Translation2d(4.572, -1.524), new Translation2d(1.524, -1.524),
        new Translation2d(0.762, -0.762), new Translation2d(0, 0.3)); //Used Autonav Waypoint Calculator Sheet


        waypointSlalomStrings = waypointsSlalom.stream().map((o) -> o.toString()).collect(toList());
        // An example trajectory to follow. All units in meters.
        slalom = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startSlalom,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsSlalom,
                // End 3 meters straight ahead of where we started, facing forward
                finishSlalom,
                // Pass config
                forwardConfig);

        
        //Bounce1
        final List<Translation2d> waypointsBounce1 = List.of(new Translation2d(0.762, 0.762)); //Used Autonav Waypoint Calculator Sheet


        waypointBounce1Strings = waypointsBounce1.stream().map((o) -> o.toString()).collect(toList());
        // An example trajectory to follow. All units in meters.
        bounceCourse1 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBounce1,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsBounce1,
                // End 3 meters straight ahead of where we started, facing forward
                finishBounce1,
                // Pass config
                forwardConfig);


        //Bounce2
        final List<Translation2d> waypointsBounce2 = List.of(new Translation2d(0.762, 0.762), new Translation2d(1.524, -0.762),
        new Translation2d(2.286, 1.524), new Translation2d(3.048, -0.762)); //Used Autonav Waypoint Calculator Sheet


        waypointBounce2Strings = waypointsBounce2.stream().map((o) -> o.toString()).collect(toList());
        // An example trajectory to follow. All units in meters.
        bounceCourse2 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBounce2,
                 // Pass through these two interior waypoints, making an 's' curve path
                waypointsBounce2,
                // End 3 meters straight ahead of where we started, facing forward
                finishBounce2,
                // Pass config
                reverseConfig);


        //Bounce3
        final List<Translation2d> waypointsBounce3 = List.of(new Translation2d(3.048, -0.762),
        new Translation2d(3.81, -1.524), new Translation2d(4.572, -1.524), new Translation2d(5.334, -0.762)); //Used Autonav Waypoint Calculator Sheet


        waypointBounce3Strings = waypointsBounce3.stream().map((o) -> o.toString()).collect(toList()); 
        // An example trajectory to follow. All units in meters.
        bounceCourse3 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBounce3,
                // Pass through these two interior waypoints, making an 's' curve path
                waypointsBounce3,
                // End 3 meters straight ahead of where we started, facing forward
                finishBounce3,
                // Pass config
                forwardConfig);


        //Bounce4
        final List<Translation2d> waypointsBounce4 = List.of(new Translation2d(5.334, -0.762), 
        new Translation2d(6.096, 0.305)); //Used Autonav Waypoint Calculator Sheet


        waypointBounce4Strings = waypointsBounce4.stream().map((o) -> o.toString()).collect(toList());
        // An example trajectory to follow. All units in meters.
        bounceCourse4 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startBounce4,
                 // Pass through these two interior waypoints, making an 's' curve path
                waypointsBounce4,
                // End 3 meters straight ahead of where we started, facing forward
                finishBounce4,
                // Pass config
                reverseConfig);
    }

    public List<Trajectory> getBarrelRacer() {
        final List<Trajectory> barrelRacerTrajectories = new ArrayList<Trajectory>();
        BadLog.createValue("Trajectory/Initial Desired Pose", ""+startBarrel);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finishBarrel);
        BadLog.createValue("Trajectory/Course", "Barrel Racer");
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointBarrelStrings));
        barrelRacerTrajectories.add(barrelRacer);
        return barrelRacerTrajectories;
    }

    public List<Trajectory> getSlalom() {
        final List<Trajectory> slalomTrajectories = new ArrayList<Trajectory>();
        BadLog.createValue("Trajectory/Initial Desired Pose", ""+startSlalom);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finishSlalom);
        BadLog.createValue("Trajectory/Course", "Slalom");
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointSlalomStrings));
        slalomTrajectories.add(slalom);
        return slalomTrajectories;
    }
    public List<Trajectory> getBounce() {
        BadLog.createValue("Trajectory/Initial Desired Pose", ""+startBounce1);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finishBounce4);
        BadLog.createValue("Trajectory/Course", "Bounce");

        final List<String> waypointBounceStrings = new ArrayList<String>();
        waypointBounceStrings.addAll(waypointBounce1Strings);
        waypointBounceStrings.addAll(waypointBounce2Strings);
        waypointBounceStrings.addAll(waypointBounce3Strings);
        waypointBounceStrings.addAll(waypointBounce4Strings);
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointBounceStrings));
        final List<Trajectory> bounceCourseTrajectories = new ArrayList<Trajectory>();
        bounceCourseTrajectories.add(bounceCourse1);
        bounceCourseTrajectories.add(bounceCourse2);
        bounceCourseTrajectories.add(bounceCourse3);
        bounceCourseTrajectories.add(bounceCourse4);
        return bounceCourseTrajectories;
    }
}
