package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    WPI_TalonFX Motor;
    Boolean isDeployed;
    Solenoid solenoid;

    public IntakeSubsystem() {
        Motor = new WPI_TalonFX(IntakeConstants.MotorCANID);
        solenoid = new Solenoid(0);
        isDeployed = false;
    }

    @Override
    public void periodic() {
        solenoid.set(isDeployed);
    }

    public void spin(double speed) {
        Motor.set(speed);
    }

    public void toggle() {
        isDeployed = !isDeployed;
        System.out.println("SETTING TRUE");
        solenoid.set(isDeployed);
    }
}
