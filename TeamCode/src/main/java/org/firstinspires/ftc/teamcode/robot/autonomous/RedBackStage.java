//package org.firstinspires.ftc.teamcode.robot.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robot.init.AprilTagOpenCvCamera;
//import org.firstinspires.ftc.teamcode.robot.init.Chassis;
//import org.firstinspires.ftc.teamcode.robot.init.ColorAverageOpenCvCamera;
//
//@Autonomous
//public class RedBackStage extends LinearOpMode {
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
////        ColorAverageOpenCvCamera colorCamera = new ColorAverageOpenCvCamera(hardwareMap);
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
////        colorCamera.setAlliance("red");
////        int element_zone = colorCamera.elementDetection(telemetry);
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
//        while (!isStopRequested() && opModeIsActive()){
//            /**TIRAR PATO Y DEJAR PIXEL**/
//            chassis.goToDistance(78, 0,1);
//            armDown();
//
//            //TODO: calibrate this
//            if(element_zone == 1){
////                //center
////                chassis.goToDegrees(180, 0.5);
////                openClaw();
////            }else if(element_zone == 2){
////                //left
////                chassis.goToDegrees(160, 0.5);
////                openClaw();
////            }else{
////                //right
////                chassis.goToDegrees(200, 0.5);
////                openClaw();
////            }
//
//            closeClaw();
//            sleep(500);
//            armUp();
//
//            chassis.goToDegrees(270, 0);
//
//
//            /**DEJAR PIXEL**/
//            chassis.goToDistance(78, 30,1);
//            while((aprilPosition == "") && (time.seconds() < timePassed + 15)){
//                if(aprilPosition == ""){
////                    if(element_zone == 1){
////                        //center
////                        aprilPosition = aprilCamera.getDesiredAprilTagPosition(6);
////                        chassis.leftRun(0.5);
////                        chassis.postCurrentPosition();
////                    }else if(element_zone == 2){
////                        //left
////                        aprilPosition = aprilCamera.getDesiredAprilTagPosition(5);
////                        chassis.leftRun(0.5);
////                        chassis.postCurrentPosition();
////                    }else{
////                        //right
////                        aprilPosition = aprilCamera.getDesiredAprilTagPosition(4);
////                        chassis.leftRun(0.5);
////                        chassis.postCurrentPosition();
////                    }
//                }else{
//                    chassis.stopChassis();
//                    chassis.postCurrentPosition();
//                }
//
//                if(time.seconds() < timePassed + 3){
//                    drawSlidersOut(0.5);
//                }else{
//                    stopSlidersOut();
//                }
//                if(time.seconds() < timePassed + 10){
//                    jackUp(1);
//                }else{
//                    stopJack();
//                }
//            }
//            chassis.stopChassis();
//            stopJack();
//            stopSlidersOut();
//            while(time.seconds() < timePassed + 3){
//                chassis.forward(-0.3);
//                chassis.postCurrentPosition();
//            }
//            boxDown();
//            sleep(500);
//            outOpen();
//            sleep(200);
//
//            outClose();
//            boxUp();
//
//            /**STOP AND RESET POSITIONS**/
//            aprilPosition = "";
//            timePassed = time.seconds();
//            while(time.seconds() < timePassed + 10){
//                if(time.seconds() < timePassed + 3){
//                    insertSlidersOut(0.5);
//                }else{
//                    stopSlidersOut();
//                }
//                if(time.seconds() < timePassed + 10){
//                    jackDown(1);
//                }else{
//                    stopJack();
//                }
//            }
//
//            /**ESTACIONAR**/
//            chassis.goToDistance(10, 30,1);
//            chassis.goToDistance(10,50,0.5);
//            chassis.stopChassis();
//
//            sleep(30000);
//        }
//
//    }
////    public void closeClaw(){
////        clawr.setPosition(0);
////        clawl.setPosition(0.5);
////    }
////    public void openClaw(){
////        clawr.setPosition(0.5);
////        clawl.setPosition(0);
////    }
////
////    public void armDown(){
////        armL.setPosition(1);
////        armR.setPosition(0);
////    }
////    public void armUp(){
////        armL.setPosition(0.17);
////        armR.setPosition(0.83);
////    }
////
////    public void jackUp(double power){
////        jackLeft.setPower(power);
////        jackRight.setPower(power);
////    }
////    public void jackDown(double power){
////        jackLeft.setPower(-power);
////        jackRight.setPower(-power);
////    }
////    public void stopJack(){
////        jackLeft.setPower(0);
////        jackRight.setPower(0);
////    }
////
////    public void drawSlidersOut(double power){
////        sliderOutLeft.setPower(-power);
////        sliderOutRight.setPower(power);
////    }
////    public void insertSlidersOut(double power){
////        sliderOutLeft.setPower(power);
////        sliderOutRight.setPower(-power);
////    }
////    public void stopSlidersOut(){
////        sliderOutLeft.setPower(0);
////        sliderOutRight.setPower(0);
////    }
////
////    public void boxUp(){
////        outakeLeft.setPosition(0.70);
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

