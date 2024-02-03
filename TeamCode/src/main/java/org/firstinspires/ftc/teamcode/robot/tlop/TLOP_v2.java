package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP_v2 extends LinearOpMode {
    //INTAKE
    private Servo armL, armR;
    private Servo clawl, clawr;
//    private Servo claw;

    //OUTAKE
    private DcMotor sliderOutLeft, sliderOutRight;
    private Servo outakeLeft, outakeRight;
    private CRServo jackLeft, jackRight;
    private Servo outake;

    //MISC
    private DcMotor sliderHangLeft, sliderHangRight;
    private Servo plane;


    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        //INTAKE
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        clawr = hardwareMap.get(Servo.class, "clawr");
        clawl = hardwareMap.get(Servo.class, "clawl");
//        claw = hardwareMap.get(Servo.class, "claw");

        //OUTAKE
        sliderOutLeft = hardwareMap.get(DcMotor.class, "sol");
        sliderOutRight = hardwareMap.get(DcMotor.class, "sor");
        outakeLeft = hardwareMap.get(Servo.class, "outl");
        outakeRight = hardwareMap.get(Servo.class, "outr");
        jackLeft = hardwareMap.get(CRServo.class, "jackl");
        jackRight = hardwareMap.get(CRServo.class, "jackr");
        outake = hardwareMap.get(Servo.class, "out");

        //MISC
        sliderHangLeft = hardwareMap.get(DcMotor.class, "shl");
        sliderHangRight = hardwareMap.get(DcMotor.class, "shr");
        plane = hardwareMap.get(Servo.class, "plane");


        /**Initial setup**/
        outakeLeft.setPosition(0.3);
        outakeRight.setPosition(0.8);
        plane.setPosition(0);
        //baja arm

        waitForStart();

        while (!isStopRequested()) {

            /**CHASSIS**/
            chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /**INTAKE**/
            if(gamepad1.right_bumper){
//                //abre claw
//                claw.setPosition(0);
                clawr.setPosition(0);
                clawl.setPosition(0.5);
            }else if(gamepad1.left_bumper){
                //cierra claw
//                claw.setPosition(0.5);
                clawr.setPosition(0.5);
                clawl.setPosition(0);
            }
            if(gamepad1.right_trigger > 0.2){
                //sube arm
                armL.setPosition(1);
                armR.setPosition(0);
//                claw.setPosition(0);
                clawr.setPosition(0);
                clawl.setPosition(0.5);
            }else if(gamepad1.left_trigger > 0.2){
                //baja arm
                armL.setPosition(0.17);
                armR.setPosition(0.83);
//                claw.setPosition(0);
                clawr.setPosition(0);
                clawl.setPosition(0.5);
            }

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
            if(gamepad2.right_trigger > 0.2){
                outakeLeft.setPosition(0.09);
                outakeRight.setPosition(0.81);
            }else if(gamepad2.right_bumper){
                outakeLeft.setPosition(0.5);
                outakeRight.setPosition(0.65);
            }else{
                outakeLeft.setPosition(0.71);
                outakeRight.setPosition(0.24);
            }
            if(gamepad2.left_trigger > 0.2){
                outake.setPosition(0.75);
            }else{
                outake.setPosition(0.25);
            }
            if(gamepad2.dpad_up){
                //jack goes up
                jackLeft.setPower(1);
                jackRight.setPower(1);
            }else if(gamepad2.dpad_down){
                //jack goes down
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

            if(gamepad1.x){
                plane.setPosition(0.5);
            }else{
                plane.setPosition(0);
            }

            chassis.postCurrentPosition();
            telemetry.addData("Orientation", chassis.getCurrentPosition().getOrientation());
            telemetry.addData("x", chassis.getCurrentPosition().getXPosition());
            telemetry.addData("y", chassis.getCurrentPosition().getYPosition());
            telemetry.update();
        }
    }
}
