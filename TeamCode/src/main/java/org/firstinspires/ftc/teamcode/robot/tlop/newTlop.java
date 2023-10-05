package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.odometry.position.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Encoder;

@TeleOp
public class newTlop extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private double EncoderX;
    private double EncoderY;
    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        waitForStart();
        while(!isStopRequested()){
            EncoderX = Encoder.getEncoder(leftFront);
            EncoderY = Encoder.getEncoder(rightBack);
            telemetry.addData("X", EncoderX);
            telemetry.addData("Y", EncoderY);
            telemetry.update();
        }
    }
}
