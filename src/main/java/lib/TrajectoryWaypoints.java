package lib;

public class TrajectoryWaypoints {
    PoseInfo start;
    PoseInfo finish;
    double[][] waypoints;
    boolean reversed;

    class PoseInfo {
        double x;
        double y;
        double theta;
    }
}
