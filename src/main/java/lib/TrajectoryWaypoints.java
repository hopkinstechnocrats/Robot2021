package lib;

public class TrajectoryWaypoints {
    PoseInfo start;
    PoseInfo finish;
    double[][] waypoints;
    boolean reversed;

    public TrajectoryWaypoints() {
        start = new PoseInfo();
        finish = new PoseInfo();
    }

    class PoseInfo {
        double x;
        double y;
        double theta;
        public PoseInfo() {
            x = 0;
            y = 0;
            theta = 0;
        }
    }
}
