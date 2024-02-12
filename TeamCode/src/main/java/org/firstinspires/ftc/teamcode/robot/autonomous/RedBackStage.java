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
//import org.firstinspires.ftc.teamcode.robot.init.cameras.AprilTagOpenCvCamera;
//import org.firstinspires.ftc.teamcode.robot.init.Chassis;
//import org.firstinspires.ftc.teamcode.robot.init.cameras.ColorAverageOpenCvCamera;
//
//@Autonomous(name="Red Backstage", group="1 Auto Red")
//public class RedBackStage extends LinearOpMode {
//    //AUTONOMOUS VARIABLES
//    String aprilPosition = "";
//    double timePassed = 0;
//
//    public void runOpMode(){
//        Chassis chassis =  new Chassis(hardwareMap, telemetry);
//        ColorAverageOpenCvCamera colorCamera = new ColorAverageOpenCvCamera(hardwareMap, telemetry);
//        colorCamera.setAlliance("red");
//        int element_zone = colorCamera.elementDetection(telemetry);
//
//        telemetry.update();
//
//        closeClaw();
//        boxUp();
//        outClose();
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
//                //center
//                chassis.goToDegrees(180, 0.5);
//                openClaw();
//            }else if(element_zone == 2){
//                //left
//                chassis.goToDegrees(160, 0.5);
//                openClaw();
//            }else{
//                //right
//                chassis.goToDegrees(200, 0.5);
//                openClaw();
//            }
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
//                    if(element_zone == 1){
//                        //center
//                        aprilPosition = aprilCamera.getDesiredAprilTagPosition(6);
//                        chassis.leftRun(0.5);
//                        chassis.postCurrentPosition();
//                    }else if(element_zone == 2){
//                        //left
//                        aprilPosition = aprilCamera.getDesiredAprilTagPosition(5);
//                        chassis.leftRun(0.5);
//                        chassis.postCurrentPosition();
//                    }else{
//                        //right
//                        aprilPosition = aprilCamera.getDesiredAprilTagPosition(4);
//                        chassis.leftRun(0.5);
//                        chassis.postCurrentPosition();
//                    }
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
//}
