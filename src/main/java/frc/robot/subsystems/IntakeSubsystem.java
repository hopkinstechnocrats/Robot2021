package frc.robot.subsystems;
import javax.sound.midi.VoiceStatus;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    WPI_TalonFX Motor;

    public IntakeSubsystem() {
        Motor = new WPI_TalonFX(IntakeConstants.MotorCANID);
    }

    public void spin(double speed) {
        Motor.set(speed);
    }
}
