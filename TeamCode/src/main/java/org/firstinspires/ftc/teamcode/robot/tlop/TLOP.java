package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    private DcMotor sliderFront;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, imu, hardwareMap);

        sliderFront = hardwareMap.get(DcMotor.class, "sF");

        waitForStart();

        while (!isStopRequested()){
            if((gamepad1.left_stick_y > 0.2)||(gamepad1.left_stick_y < -0.2)){
                chassis.forward(gamepad1.left_stick_y);
            }

            if((gamepad1.right_stick_x > 0.2)||(gamepad1.right_stick_x < -0.2)){
                chassis.turnRight(gamepad1.right_stick_x);
            }

            if((gamepad1.left_stick_x > 0.2)||(gamepad1.left_stick_x < -0.2)){
                chassis.leftRun(gamepad1.left_stick_x);
            }
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    private DcMotor sliderFront;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, imu, hardwareMap);

        sliderFront = hardwareMap.get(DcMotor.class, "sF");

        waitForStart();

        while (!isStopRequested()){
            if((gamepad1.left_stick_y > 0.2)||(gamepad1.left_stick_y < -0.2)){
                chassis.forward(gamepad1.left_stick_y);
            }

            if((gamepad1.right_stick_x > 0.2)||(gamepad1.right_stick_x < -0.2)){
                chassis.turnRight(gamepad1.right_stick_x);
            }

            if((gamepad1.left_stick_x > 0.2)||(gamepad1.left_stick_x < -0.2)){
                chassis.leftRun(gamepad1.left_stick_x);
            }
            chassis.stopChassis();

            if(gamepad1.right_bumper){
                sliderFront.setPower(1);
            }else if(gamepad1.left_bumper){
                sliderFront.setPower(-1);
            }else{
                sliderFront.setPower(0);
            }
        }
    }
}
