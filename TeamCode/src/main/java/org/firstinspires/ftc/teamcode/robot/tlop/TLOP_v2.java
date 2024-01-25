package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP_v2 extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;

    private DcMotor sliderOutLeft, sliderOutRight;
    private DcMotor sliderHangLeft, sliderHangRight;
    private Servo outakeLeft, outakeRight;
    private CRServo jackLeft, jackRight;


    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, hardwareMap, telemetry);
        chassis.initChassis();

        sliderOutLeft = hardwareMap.get(DcMotor.class, "sol");
        sliderOutRight = hardwareMap.get(DcMotor.class, "sor");

        sliderHangLeft = hardwareMap.get(DcMotor.class, "shl");
        sliderHangRight = hardwareMap.get(DcMotor.class, "shr");

        outakeLeft = hardwareMap.get(Servo.class, "outl");
        outakeRight = hardwareMap.get(Servo.class, "outr");

        jackLeft = hardwareMap.get(CRServo.class, "jackl");
        jackRight = hardwareMap.get(CRServo.class, "jackr");

        //Initial setup
        outakeLeft.setPosition(0.3);
        outakeRight.setPosition(0.8);

        waitForStart();

        while (!isStopRequested()) {

            /**CHASSIS**/
            chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /**INTAKE**/

            /**OUTAKE**/
            if((gamepad2.right_stick_y > 0.2) || (gamepad2.right_stick_y < -0.2)){
                if(gamepad2.right_stick_y > 0.2){
                    sliderOutLeft.setPower(-gamepad2.right_stick_y);
                    sliderOutRight.setPower(gamepad2.right_stick_y);

                }else if(gamepad2.right_stick_y < -0.2){
                    sliderOutLeft.setPower(-gamepad2.right_stick_y);
                    sliderOutRight.setPower(gamepad2.right_stick_y);
                }
            }else{
                sliderOutLeft.setPower(0);
                sliderOutRight.setPower(0);
            }
            if(gamepad2.right_trigger > 1){
                outakeLeft.setPosition(0.8);
                outakeRight.setPosition(0.3);
            }else{
                outakeLeft.setPosition(0.3);
                outakeRight.setPosition(0.8);
            }
            if(gamepad2.dpad_up){
                jackLeft.setPower(1);
                jackRight.setPower(1);
            }else if(gamepad2.dpad_down){
                jackLeft.setPower(-1);
                jackRight.setPower(-1);
            }else{
                jackLeft.setPower(0);
                jackRight.setPower(0);
            }


            /**MISC**/
            if((gamepad2.left_stick_y > 0.2) || (gamepad2.left_stick_y < -0.2)){
                if(gamepad2.left_stick_y > 0.2){
                    sliderHangLeft.setPower(gamepad2.left_stick_y);
                    sliderHangRight.setPower(gamepad2.left_stick_y);

                }else if(gamepad2.left_stick_y < -0.2){
                    sliderHangLeft.setPower(gamepad2.left_stick_y);
                    sliderHangRight.setPower(gamepad2.left_stick_y);
                }
            }else{
                sliderHangLeft.setPower(0);
                sliderHangRight.setPower(0);
            }

            telemetry.update();
        }
    }
}
