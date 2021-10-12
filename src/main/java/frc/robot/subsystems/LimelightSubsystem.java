package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Limelight;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {

    }

    public double getAngleFromTarget() {
        return Limelight.getAngleFromTarget();
    }

    public double getDistanceFromTarget() {
        return Limelight.getDistanceFromTarget();
    }

    public boolean isTargetVisible() {
        return Limelight.isTargetVisible();
    }
}

