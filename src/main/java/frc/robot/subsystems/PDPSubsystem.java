package frc.robot.subsystems;


import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Loggable;

public class PDPSubsystem extends SubsystemBase implements Loggable {
    private final PowerDistributionPanel PDP;

    public PDPSubsystem() {
        PDP = new PowerDistributionPanel();
    }

    public void logInit() {
        for (int i = 0; i <= 15; i++) {
            final int j = i;
            BadLog.createTopic("PDP/PDP" + j + " Current", "Amperes", () -> PDP.getCurrent(j));
        }
        BadLog.createTopic("PDP/PDP Temp", "Celsius", PDP::getTemperature);
        BadLog.createTopic("PDP/PDP Total Current", "Amperes", PDP::getTotalCurrent);
        BadLog.createTopic("PDP/PDP Total Energy", "Joules", PDP::getTotalEnergy);
        BadLog.createTopic("PDP/PDP Total Power", "Watts", PDP::getTotalPower);
        BadLog.createTopic("PDP/PDP Input Voltage", "Volts", PDP::getVoltage);
    }

}

