package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PreLaunchConstants;
import lib.MotorFaultLogger;

public class PreLaunchSubsystem extends SubsystemBase {
    final WPI_TalonFX Motor1;

    public PreLaunchSubsystem() {
        Motor1 = new WPI_TalonFX(PreLaunchConstants.Motor1CANID);
        MotorFaultLogger.getInstance().add("PrelaunchMotor", Motor1);
    }

    public void spin(double speed) {
        Motor1.set(speed);
    }
}
