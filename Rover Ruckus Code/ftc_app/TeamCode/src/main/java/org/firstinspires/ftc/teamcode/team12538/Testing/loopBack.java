package org.firstinspires.ftc.teamcode.team12538.Testing;

public class loopBack{
    private static double rot = 0.0;
    private static double previous = 0.0;
    private static double loopBackcone = 90;//In my own experience a 90 degree loopback window works nicely, although you can tune to your preferences

    //todo: find a way to determine heading from input voltages
    public static double getRotation(double heading){
        if ((heading - previous) < loopBackcone - 360){
            rot += (360 + (heading - previous)) / 360;
        } else if((heading - previous) > 360 - loopBackcone){
            rot -= (360 + (previous - heading)) / 360;
        }
        else{
            rot += (heading - previous) / 360;
        }
        previous = heading;
        return rot;
    }
}
