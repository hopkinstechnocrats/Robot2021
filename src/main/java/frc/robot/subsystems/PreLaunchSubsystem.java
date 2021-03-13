package frc.robot.subsystems;

import javax.sound.midi.VoiceStatus;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PreLaunchConstants;
import frc.robot.Constants.PreLaunchConstraints;

public class PreLaunchSubsystem extends SubsystemBase{
    WPI_TalonFX Motor1;

    public PreLaunchSubsystem() {
        Motor1 = new WPI_TalonFX(PreLaunchConstants.Motor1CANID);
    }

    public void spin(double speed) {
        Motor1.set(ControlMode.Velocity, speed);
    }
}
