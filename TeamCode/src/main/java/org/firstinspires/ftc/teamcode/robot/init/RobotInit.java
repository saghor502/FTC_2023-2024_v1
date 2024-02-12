package org.firstinspires.ftc.teamcode.robot.init;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotInit {
    //INTAKE
    public Servo armL, armR;
    public Servo clawl, clawr;
//    private Servo claw;

    //OUTAKE
    public DcMotor sliderOutLeft, sliderOutRight;
    public Servo outakeLeft;
    public CRServo jackLeft, jackRight;
    public Servo outake;

    //MISC
    public DcMotor sliderHangLeft, sliderHangRight;
    private Servo cameraServo;

    public RobotInit(HardwareMap hardwareMap, Telemetry telemetry){
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
        jackLeft = hardwareMap.get(CRServo.class, "jackl");
        jackRight = hardwareMap.get(CRServo.class, "jackr");
        outake = hardwareMap.get(Servo.class, "out");

        //MISC
        sliderHangLeft = hardwareMap.get(DcMotor.class, "shl");
        sliderHangRight = hardwareMap.get(DcMotor.class, "shr");
        cameraServo = hardwareMap.get(Servo.class, "cameraS");
    }

    public void cameraRight(){
        cameraServo.setPosition(0.75);
    }
    public void cameraMiddle(){
        cameraServo.setPosition(0.65);
    }
    public void cameraLeft(){
        cameraServo.setPosition(0.55);
    }
    public void storeCamera(){
        cameraServo.setPosition(1);
    }

    public void closeClaw(){
        clawr.setPosition(0);
        clawl.setPosition(0.5);
    }
    public void openClaw(){
        clawr.setPosition(0.5);
        clawl.setPosition(0);
    }

    public void armDown(){
        armL.setPosition(1);
        armR.setPosition(0);
    }
    public void armUp(){
        armL.setPosition(0.17);
        armR.setPosition(0.83);
    }

    public void jackUp(double power){
        jackLeft.setPower(power);
        jackRight.setPower(power);
    }
    public void jackDown(double power){
        jackLeft.setPower(-power);
        jackRight.setPower(-power);
    }
    public void stopJack(){
        jackLeft.setPower(0);
        jackRight.setPower(0);
    }

    public void drawSlidersOut(double power){
        sliderOutLeft.setPower(-power);
        sliderOutRight.setPower(power);
    }
    public void insertSlidersOut(double power){
        sliderOutLeft.setPower(power);
        sliderOutRight.setPower(-power);
    }
    public void stopSlidersOut(){
        sliderOutLeft.setPower(0);
        sliderOutRight.setPower(0);
    }

    public void hangSlidersOut(double power){
        sliderHangLeft.setPower(power);
        sliderHangRight.setPower(power);
    }
    public void hangSlidersIn(double power){
        sliderHangLeft.setPower(-power);
        sliderHangRight.setPower(-power);
    }
    public void stopHangSliders(){
        sliderHangLeft.setPower(0);
        sliderHangRight.setPower(0);

    }

    public void boxUp(){
        outakeLeft.setPosition(0.95);
    }
    public void boxMiddle(){
        outakeLeft.setPosition(0.65);
    }
    public void boxDown(){
        outakeLeft.setPosition(0.4);
    }

    public void outOpen(){
        outake.setPosition(0.75);
    }
    public void outClose(){
        outake.setPosition(0.35);
    }
}
