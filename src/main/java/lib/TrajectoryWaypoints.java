package lib;

public class TrajectoryWaypoints {
    class PoseInfo {
        double x;
        double y;
        double theta;
    }

    PoseInfo start;
    PoseInfo finish;
    double[][] waypoints;
    boolean reversed;
}
