package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX motor;
    private double lastSpeed;

    public IntakeSubsystem(int motorID) {
        this.motor = new WPI_TalonFX(motorID);
        this.lastSpeed = 0;
    }

    public void spin() {
        motor.set(lastSpeed);
    } // Potential idea: Call spin() on teleop Periodic, and modify lastSpeed when
      // stuff is pressed.

    public void spin(double newSpeed) {
        this.setSpeed(newSpeed);
        this.spin();
    }

    public void setSpeed(double newSpeed) {
        this.lastSpeed = newSpeed;
    }

    public double getSpeed() {
        return this.lastSpeed;
    }
}
