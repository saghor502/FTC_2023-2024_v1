package org.firstinspires.ftc.teamcode.odometry.position;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {
    public static double getEncoder(DcMotor Encoder){
        double Resolution = Encoder.getCurrentPosition();
        return Resolution;
    }
}
