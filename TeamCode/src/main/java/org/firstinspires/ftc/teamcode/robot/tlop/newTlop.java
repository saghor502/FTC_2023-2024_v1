package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.odometry.position.Encoder;
@TeleOp
public class newTlop extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private double EncoderX;
    private double EncoderY;

    //--------Config
    public static double RWYCoordinate = 10;
    public static double
    //--------------
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

            if (gamepad1.right_stick_y < 0.1){
                forward(gamepad1.right_stick_y);
            } else if (gamepad1.right_stick_y < 0.1){
                forward(-gamepad1.right_stick_y);
            } else {
                forward(0);
            }
            if (gamepad1.left_stick_x < 0.1){
                turn(gamepad1.left_stick_x);
            } else if (gamepad1.left_stick_x < 0.1){
                turn(-gamepad1.left_stick_x);
            } else {
                turn(0);
            }
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
