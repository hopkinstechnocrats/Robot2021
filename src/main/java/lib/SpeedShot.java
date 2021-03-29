package lib;

import java.util.HashMap;

public class SpeedShot {
    public static double speed;
    public static HashMap<Double, Double> speedMap = new HashMap<Double, Double>();

    public SpeedShot() {
        
    speedMap.put(10.0,2.0);//Distance, Speed
    }

    public double getLauncherDesiredSpeed(double distance){
        speed = LinearInterpolation.getLinearInterpolation(speedMap, distance);
        return speed;
    }
}
