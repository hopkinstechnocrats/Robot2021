package lib;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
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
        Pose2d start = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d finish = new Pose2d(-1.5, 0.3, new Rotation2d(Math.PI));
        List<Translation2d> waypoints = List.of(new Translation2d(2.286, 0.3048), new Translation2d(3.3528, -0.762),
        new Translation2d(2.286, -1.8288), new Translation2d(1.2192, -0.762),
        new Translation2d(2.286, 0.3048), new Translation2d(4.572, -0.3048),
        new Translation2d(5.6388, 0.762), new Translation2d(4.572, 1.8288),
        new Translation2d(3.5052, 0.762), new Translation2d(4.572, -0.3048),
        new Translation2d(6.096, -1.8288), new Translation2d(7.1628, -0.762),
        new Translation2d(6.096, 0.3048)); //Used Autonav Waypoint Calculator Sheet



        BadLog.createValue("Trajectory/Initial Desired Pose", ""+start);
        BadLog.createValue("Trajectory/Final Desired Pose", ""+finish);

        List<String> waypointStrings = waypoints.stream().map((o) -> o.toString()).collect(toList());
		private final TrajectoryConfig config;
        BadLog.createValue("Trajectory/Interior Waypoints", String.join("", waypointStrings));
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                start,
                // Pass through these two interior waypoints, making an 's' curve path
                waypoints,
                // End 3 meters straight ahead of where we started, facing forward
                finish,
                // Pass config
                config);
    }
}
