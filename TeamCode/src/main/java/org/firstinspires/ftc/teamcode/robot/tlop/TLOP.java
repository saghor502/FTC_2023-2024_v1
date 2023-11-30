package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Servo articulation1, articulation2, middleArt, claw;
    private CRServo cat;
    private Servo outake1, outake2;
    private DcMotor upr, upl;
    private DcMotor slidervert;
    private static BNO055IMU imu;

    private DcMotor sliderFront;
    private boolean positionMA = false;
    private boolean positionArt = false;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, hardwareMap, telemetry);

        sliderFront = hardwareMap.get(DcMotor.class, "sF");
        slidervert = hardwareMap.get(DcMotor.class, "sV");

        articulation1 = hardwareMap.get(Servo.class, "a1");
        articulation2 = hardwareMap.get(Servo.class, "a2");
        middleArt =  hardwareMap.get(Servo.class, "middleArt");
        claw =  hardwareMap.get(Servo.class, "claw");
        cat =  hardwareMap.get(CRServo.class, "cat");

        outake1 =  hardwareMap.get(Servo.class, "out1");
        outake2 =  hardwareMap.get(Servo.class, "out2");

        upr =  hardwareMap.get(DcMotor.class, "ur");
        upl =  hardwareMap.get(DcMotor.class, "ul");



        waitForStart();

        while (!isStopRequested()) {

            /**CHASSIS**/
            if((gamepad1.left_stick_y > 0.2) || (gamepad1.left_stick_y < -0.2) ||
                    (gamepad1.right_stick_x > 0.2) || (gamepad1.right_stick_x < -0.2) ||
                    (gamepad1.left_stick_x > 0.2) || (gamepad1.left_stick_x < -0.2)){

                if ((gamepad1.left_stick_y > 0.2) || (gamepad1.left_stick_y < -0.2)) {
                    chassis.forward(gamepad1.left_stick_y);
                }
                if ((gamepad1.right_stick_x > 0.2) || (gamepad1.right_stick_x < -0.2)) {
                    chassis.turnRight(gamepad1.right_stick_x);
                }
                if ((gamepad1.left_stick_x > 0.2) || (gamepad1.left_stick_x < -0.2)) {
                    chassis.leftRun(gamepad1.left_stick_x);
                }
            }else{
                chassis.stopChassis();
            }

            /**INTAKE**/
            if (gamepad1.right_bumper) {
                sliderFront.setPower(1);
            } else if (gamepad1.left_bumper) {
                sliderFront.setPower(-1);
            } else {
                sliderFront.setPower(0);
            }

            if(gamepad1.left_trigger > 0.2){
                claw.setPosition(0);
            }else if(gamepad1.right_trigger > 0.2){
                claw.setPosition(1);
            }

            if(gamepad1.dpad_up){
                positionMA = false;
            }else if(gamepad1.dpad_down){
                positionMA = true;
            }

            if(positionMA){
                middleArt.setPosition(0.95);
            }else{
                middleArt.setPosition(0.8);
            }

            if(gamepad1.y){
                positionArt = true;
            }else if(gamepad1.x){
                positionArt = false;
            }

            if(positionArt){
                articulation1.setPosition(0.4);
                articulation2.setPosition(0.4);
            }else{
                articulation1.setPosition(0.775);
                articulation2.setPosition(0.775);
            }

            /**OUTAKE*/
            if(gamepad2.dpad_up){
                cat.setPower(0.5);
            }else if(gamepad2.dpad_down){
                cat.setPower(-0.5);
            }else{
                cat.setPower(0);
            }

            if (gamepad2.right_stick_y > 0.2) {
                slidervert.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < -0.2) {
                slidervert.setPower(gamepad2.right_stick_y);
            } else {
                slidervert.setPower(0);
            }

            if(gamepad2.right_bumper){
                outake1.setPosition(0.5);
            }else if(gamepad2.left_bumper){
                outake1.setPosition(0.15);
            }

            if(gamepad2.right_trigger > 0.2){
                outake2.setPosition(0.4);
            }else{
                outake2.setPosition(0.75);
            }

            /**WEIRD MECHANISMS XD**/

            double mult1 = 0.5;

            if (gamepad2.left_stick_y > 0.2) {
                upr.setPower(gamepad2.left_stick_y * mult1);
                upl.setPower(-gamepad2.left_stick_y * mult1);
            } else if (gamepad2.left_stick_y < -0.2) {
                upr.setPower(gamepad2.left_stick_y * mult1);
                upl.setPower(-gamepad2.left_stick_y * mult1);
            } else {
                upr.setPower(0);
                upl.setPower(0);
            }

            telemetry.update();
        }
    }
}
