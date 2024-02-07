//package org.firstinspires.ftc.teamcode.robot.autonomous;
//
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robot.init.Chassis;
//import org.firstinspires.ftc.teamcode.robot.init.AprilTagOpenCvCamera;
//import org.firstinspires.ftc.teamcode.robot.init.ColorAverageOpenCvCamera;
//
//
//@Autonomous
//public class RedJustPark  extends LinearOpMode {
//    private Servo armL, armR;
//    private Servo clawl, clawr;
//
//    //OUTAKE
//    private DcMotor sliderOutLeft, sliderOutRight;
//    private Servo outakeLeft, outakeRight;
//    private CRServo jackLeft, jackRight;
//    private Servo outake;
//
//    //MISC
//    private DcMotor sliderHangLeft, sliderHangRight;
////    private Servo plane;
//    private Servo cameraServo;
//
//    //AUTONOMOUS VARIABLES
//    String aprilPosition = "";
//    double timePassed = 0;
//
//    public void runOpMode(){
//        Chassis chassis = new Chassis(hardwareMap, telemetry);
//        AprilTagOpenCvCamera aprilCamera = new AprilTagOpenCvCamera(hardwareMap, telemetry);
//        ColorAverageOpenCvCamera colorCamera = new ColorAverageOpenCvCamera(hardwareMap);
//        ElapsedTime time = new ElapsedTime();
//        //INTAKE
//        armL = hardwareMap.get(Servo.class, "armL");
//        armR = hardwareMap.get(Servo.class, "armR");
//        clawr = hardwareMap.get(Servo.class, "clawr");
//        clawl = hardwareMap.get(Servo.class, "clawl");
//
//        //OUTAKE
//        sliderOutLeft = hardwareMap.get(DcMotor.class, "sol");
//        sliderOutRight = hardwareMap.get(DcMotor.class, "sor");
//        outakeLeft = hardwareMap.get(Servo.class, "outl");
//        outakeRight = hardwareMap.get(Servo.class, "outr");
//        jackLeft = hardwareMap.get(CRServo.class, "jackl");
//        jackRight = hardwareMap.get(CRServo.class, "jackr");
//        outake = hardwareMap.get(Servo.class, "out");
//
//        //MISC
//        sliderHangLeft = hardwareMap.get(DcMotor.class, "shl");
//        sliderHangRight = hardwareMap.get(DcMotor.class, "shr");
////        plane = hardwareMap.get(Servo.class, "plane");
//        cameraServo = hardwareMap.get(Servo.class, "cameraS");
//
//
//        colorCamera.setAlliance("red");
//        int element_zone = colorCamera.elementDetection(telemetry);
//
//        telemetry.update();
//
//        closeClaw();
//        boxUp();
//        outClose();
//        cameraServo.setPosition(0.65);
//
//        time.startTime();
//
//        waitForStart();
//
//        while (!isStopRequested() && opModeIsActive()) {
//            time.reset();
//
//            chassis.goToDistance(0,10,0.5);
//
//            /**ESTACIONAR**/
//            chassis.goToDistance(50,10,0.5);
//            chassis.stopChassis();
//
//            sleep(30000);
//        }
//    }
//
//    public void closeClaw(){
//        clawr.setPosition(0);
//        clawl.setPosition(0.5);
//    }
//    public void openClaw(){
//        clawr.setPosition(0.5);
//        clawl.setPosition(0);
//    }
//
//    public void armDown(){
//        armL.setPosition(1);
//        armR.setPosition(0);
//    }
//    public void armUp(){
//        armL.setPosition(0.17);
//        armR.setPosition(0.83);
//    }
//
//    public void jackUp(double power){
//        jackLeft.setPower(power);
//        jackRight.setPower(power);
//    }
//    public void jackDown(double power){
//        jackLeft.setPower(-power);
//        jackRight.setPower(-power);
//    }
//    public void stopJack(){
//        jackLeft.setPower(0);
//        jackRight.setPower(0);
//    }
//
//    public void drawSlidersOut(double power){
//        sliderOutLeft.setPower(-power);
//        sliderOutRight.setPower(power);
//    }
//    public void insertSlidersOut(double power){
//        sliderOutLeft.setPower(power);
//        sliderOutRight.setPower(-power);
//    }
//    public void stopSlidersOut(){
//        sliderOutLeft.setPower(0);
//        sliderOutRight.setPower(0);
//    }
//
//    public void boxUp(){
//        outakeLeft.setPosition(0.70);
//        outakeRight.setPosition(0.25);
//    }
//    public void boxDown(){
//        outakeLeft.setPosition(0.5);
//        outakeRight.setPosition(0.65);
//    }
//    public void outOpen(){
//        outake.setPosition(0.75);
//    }
//    public void outClose(){
//        outake.setPosition(0.35);
//    }
//}
