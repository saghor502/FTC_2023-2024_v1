package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private Servo articulation1, articulation2;
    private DcMotor slidervert1, slidervert2;
    private static BNO055IMU imu;

    int position = 0;

    private DcMotor sliderFront;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, imu, hardwareMap, telemetry);

        sliderFront = hardwareMap.get(DcMotor.class, "sF");
        articulation1 = hardwareMap.get(Servo.class, "a1");
        articulation2 = hardwareMap.get(Servo.class, "a2");
        slidervert1 = hardwareMap.get(DcMotor.class, "sv1");
        slidervert2 = hardwareMap.get(DcMotor.class, "sv2");

        waitForStart();

        while (!isStopRequested()) {
            /**CHASSIS**/
            if ((gamepad1.left_stick_y > 0.2) || (gamepad1.left_stick_y < -0.2)) {
                chassis.forward(gamepad1.left_stick_y);
            }
            if ((gamepad1.right_stick_x > 0.2) || (gamepad1.right_stick_x < -0.2)) {
                chassis.turnRight(gamepad1.right_stick_x);
            }
            if ((gamepad1.left_stick_x > 0.2) || (gamepad1.left_stick_x < -0.2)) {
                chassis.leftRun(gamepad1.left_stick_x);
            }
            chassis.stopChassis();

            /**INTAKE**/
            if (gamepad1.right_bumper) {
                sliderFront.setPower(1);
            } else if (gamepad1.left_bumper) {
                sliderFront.setPower(-1);
            } else {
                sliderFront.setPower(0);
            }

            if(gamepad1.dpad_up){
                position = 1;
            }else if(gamepad1.dpad_down){
                position = 0;
            }else if(gamepad1.dpad_right){
                if(position == 3){
                    position = 3;
                }else{
                    position += 1;
                }
            }else if(gamepad1.dpad_left){
                if(position == 0){
                    position = 0;
                }else{
                    position -= 1;
                }
            }

            if(position == 0){
                articulationDown();
            }else if(position == 1){
                articulationMiddle2();
            }else if(position == 2){
                articulationMiddle1();
            }else if(position == 3){
                articulationUp();
            }

            /**OUTAKE**/
            if (gamepad2.right_stick_y > 0.2) {
                sliderVertical(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < -0.2) {
                sliderVertical(gamepad2.right_stick_y);
            } else {
                sliderVertical(0);
            }
        }
    }
    void sliderVertical(double power){
        slidervert1.setPower(power);
        slidervert2.setPower(power);
    }
    void articulationUp(){
        articulation1.setPosition(0.5); //TODO: view positions
        articulation2.setPosition(0.5); //TODO: view positions
    }
    void articulationMiddle1(){
        articulation1.setPosition(0.5); //TODO: view positions
        articulation2.setPosition(0.5); //TODO: view positions
    }
    void articulationMiddle2(){
        articulation1.setPosition(0.5); //TODO: view positions
        articulation2.setPosition(0.5); //TODO: view positions
    }
    void articulationDown(){
        articulation1.setPosition(0.5); //TODO: view positions
        articulation2.setPosition(0.5); //TODO: view positions
    }
}
