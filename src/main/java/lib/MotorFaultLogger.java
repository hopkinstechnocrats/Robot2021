package lib;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import java.util.Map;

public class MotorFaultLogger extends LoggableBase {
    private static MotorFaultLogger instance;
    private String Faultss;
    private Faults faults;
    private Map<String, BaseMotorController> motors;

    public static MotorFaultLogger getInstance() {
        if (instance == null) {
            instance = new MotorFaultLogger();
        }
        return instance;
    }

    static String convertFaultToStr(Faults motorFaults) {

        String returnStr = "";
        if (motorFaults.APIError) {
            returnStr += "APIError, ";
        }
        if (motorFaults.ForwardLimitSwitch) {
            returnStr += "ForwardLimitSwitch, ";
        }
        if (motorFaults.ForwardSoftLimit) {
            returnStr += "ForwardSoftLimit, ";
        }
        if (motorFaults.HardwareESDReset) {
            returnStr += "HardwareESDReset, ";
        }
        if (motorFaults.HardwareFailure) {
            returnStr += "HardwareFailure, ";
        }
        if (motorFaults.RemoteLossOfSignal) {
            returnStr += "RemoteLossOfSignal, ";
        }
        if (motorFaults.ResetDuringEn) {
            returnStr += "ResetDuringEn, ";
        }
        if (motorFaults.ReverseLimitSwitch) {
            returnStr += "ReverseLimitSwitch, ";
        }
        if (motorFaults.ReverseSoftLimit) {
            returnStr += "ReverseSoftLimit, ";
        }
        if (motorFaults.SensorOutOfPhase) {
            returnStr += "SensorOutOfPhase, ";
        }
        if (motorFaults.SensorOverflow) {
            returnStr += "SensorOverflow, ";
        }
        if (motorFaults.SupplyOverV) {
            returnStr += "SupplyOverV, ";
        }
        if (motorFaults.SupplyUnstable) {
            returnStr += "SupplyUnstable, ";
        }
        if (motorFaults.UnderVoltage) {
            returnStr += "UnderVoltage, ";
        }
        return returnStr;
    }

    public void add(Map<String, BaseMotorController> motors) {
        this.motors.putAll(motors);
    }

    @Override
    public void logInit() {
        BadLog.createTopicStr("Motor Faults", BadLog.UNITLESS, this::logPeriodicCallBack, "log");
    }

    public String logPeriodicCallBack() {
        for (String key : motors.keySet()) {
            BaseMotorController Talon = motors.get(key);
            Talon.getFaults(faults);
            String FaultsString = convertFaultToStr(faults);
            if (FaultsString != "") {
                Faultss += "{" + key + ": " + FaultsString + "}";
            }
        }
        return Faultss;
    }
}
