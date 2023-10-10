package org.firstinspires.ftc.teamcode.Odo;

public class SimplePos {
    public static double Position(double StartPos, double FinalPos){
        double desPos = FinalPos - StartPos;
        return desPos;
    }
    public static double getTurn(double X, double Y){
        double Theta = Math.atan2(X, Y);
        return Theta;
    }
    public static double getDistance(double X, double Y){
        double distance = Math.sqrt(X * X + Y + Y);
        return distance;
    }
    public static double SimpleTurn(double Δr, double Δl, double Ry, double Ly, double dx){
        double rightDistance = Δr / 8192 * 15.707963267948966192313216916398;
        double leftDistance = Δl / 8192 * 15.707963267948966192313216916398;
        double turnedRadians = rightDistance - leftDistance / Ly - Ry;
        double Err = dx - turnedRadians;
        return Err;
    }
}
