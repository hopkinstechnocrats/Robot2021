package lib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.Faults;

public class MotorFaultLogger {
    static String Faultss;
    static Faults faults;

    public void MotorFaultLogger() {
        faults = new Faults();
        Faultss = "";
    }

    public static String Logger(ArrayList<WPI_TalonFX> _fxes, ArrayList<String> names) {
    for(int i = 0; i<names.size(); i++){
        WPI_TalonFX Talon = _fxes.get(i);
        Talon.getFaults(faults);
        String MasterFaults = convertFaultToStr(faults);
    if (MasterFaults != "") {
      Faultss += "{"+ names.get(i) + ": "+MasterFaults+"}";
    }
    }   
    return Faultss;
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
    
}

