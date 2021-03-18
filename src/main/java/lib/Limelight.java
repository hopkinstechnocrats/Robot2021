package lib;

import frc.robot.Constants.LimelightConstants;

public class Limelight {
    static boolean isTargetVisible() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        return tv == 1;
    }

    static double getDistanceFromTarget() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return (limelightConstants.powerPortDistanceToGround-limelightConstants.limelightDistanceToGround) / tan(ty+LimelightConstants.mountingAngle);
    }

    static double getAngleFromTarget() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
}