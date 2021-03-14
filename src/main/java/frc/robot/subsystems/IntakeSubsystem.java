package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX motor;

    public IntakeSubsystem(int motorID) {
        this.motor = new WPI_TalonFX(motorID);
    }

    public void spin(double newSpeed) {
        motor.set(newSpeed);
    }
}
