package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import lib.MotorFaultLogger;

public class HoodSubsystem extends SubsystemBase {
    Boolean isDeployed;
    final Solenoid solenoid;

    public HoodSubsystem() {
        solenoid = new Solenoid(1);
        isDeployed = false;
    }

    @Override
    public void periodic() {
        solenoid.set(isDeployed);
    }

    public void toggle() {
        isDeployed = !isDeployed;
        System.out.println("SETTING TRUE");
        solenoid.set(isDeployed);
    }
}
