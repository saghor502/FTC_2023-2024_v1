package org.firstinspires.ftc.teamcode.odometry.position;

public class Position {
    private static double xPosition;
    private static double yPosition;
    private static double orientation;

    public Position(double xPosition, double yPosition, int orientation) {
        Position.xPosition = xPosition;
        Position.yPosition = yPosition;
        Position.orientation = orientation;
    }

    public static double getXPosition(){
        return xPosition;
    }
    public static double getYPosition(){
        return yPosition;
    }
    public static double getOrientation(){
        return orientation;
    }

    public static void setXPosition(double xposition){Position.xPosition = xposition;}
    public static void setYPosition(double yposition){Position.yPosition = yposition;}
    public static void setOrientation(double orientation){Position.orientation = orientation;}
}
