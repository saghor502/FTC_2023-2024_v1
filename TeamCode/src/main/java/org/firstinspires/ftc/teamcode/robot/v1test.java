package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.functions.OdoMath;

@Autonomous
public class v1test extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private AnalogInput EncoderX;

    double EncoderTickX = EncoderX.getMaxVoltage(); //Get Encoder 
    @Override
    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        EncoderX = hardwareMap.get(AnalogInput.class, "x");
        waitForStart();

        while (!isStopRequested()){
            double DistanceX = OdoMath.getDistance(EncoderTickX);
        }
    }
}
