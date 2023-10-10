package org.firstinspires.ftc.teamcode.robot.autonomous.AUTOtest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.odometry.position.Encoder;

import org.firstinspires.ftc.teamcode.Odo.SimplePos;
import org.firstinspires.ftc.teamcode.odometry.position.Position;

@Autonomous
public class SimplePosAuto extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private double EncoderX; //R
    private double EncoderY;
    private double Encoderθ; //L

    //--------Config
    public static double RWYCoordinate = 10;
    public static double LWYCoordinate = 10;

    //--------Map

    private double YdesiredPos = SimplePos.Position(0.50, 2.00);
    private double XdesiredPos = SimplePos.Position(0.50, 2.00);

    //Constants

    private double turnAngle = SimplePos.getTurn(XdesiredPos, YdesiredPos);
    private double desiredD = SimplePos.getDistance(XdesiredPos, YdesiredPos);

    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        waitForStart();
        while(!isStopRequested()){

            double TurnError = SimplePos.SimpleTurn(EncoderX, Encoderθ, RWYCoordinate, LWYCoordinate, turnAngle);

            EncoderX = Encoder.getEncoder(leftFront);
            EncoderY = Encoder.getEncoder(rightBack);
            Encoderθ = Encoder.getEncoder(leftBack);
            telemetry.addData("X", EncoderX);
            telemetry.addData("Y", EncoderY);
            telemetry.addData("θErr", TurnError);
            telemetry.update();



        }
    }
    public void forward(double pow){
        leftFront.setPower(pow);
        rightFront.setPower(-pow);
        rightBack.setPower(-pow);
        leftBack.setPower(pow);
    }
    public void turn(double pow){
        leftFront.setPower(-pow);
        rightFront.setPower(-pow);
        rightBack.setPower(-pow);
        leftBack.setPower(-pow);
    }
}
