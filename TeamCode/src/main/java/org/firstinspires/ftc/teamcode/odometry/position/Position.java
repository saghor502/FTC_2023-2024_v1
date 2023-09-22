package org.firstinspires.ftc.teamcode.odometry.position;

public class Position {
    private static double xPosition;
    private static double yPosition;
    private static int orientation;

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
    public static int getOrientation(){
        return orientation;
    }
}
