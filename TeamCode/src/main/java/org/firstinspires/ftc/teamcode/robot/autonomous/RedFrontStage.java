package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.OpenCvCamera;


@Autonomous
public class RedFrontStage  extends LinearOpMode {
    private Servo armL, armR;
    private Servo clawl, clawr;

    //OUTAKE
    private DcMotor sliderOutLeft, sliderOutRight;
    private Servo outakeLeft, outakeRight;
    private CRServo jackLeft, jackRight;
    private Servo outake;

    //MISC
    private DcMotor sliderHangLeft, sliderHangRight;
    private Servo plane;
    private Servo cameraServo;

    //AUTONOMOUS VARIABLES
    String position = "";
    String aprilTagPosition = "";
    int aprilTagId = 8;
    double timePassed = 0;

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        OpenCvCamera camera = new OpenCvCamera(hardwareMap, telemetry);
        ElapsedTime time = new ElapsedTime();
        //INTAKE
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        clawr = hardwareMap.get(Servo.class, "clawr");
        clawl = hardwareMap.get(Servo.class, "clawl");

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
        cameraServo = hardwareMap.get(Servo.class, "cameraS");

        closeClaw();
        time.startTime();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()){
            time.reset();
            if(aprilTagPosition == ""){
                cameraServo.setPosition(0.75);
                sleep(800);
                position = camera.getDesiredAprilTagPosition(aprilTagId);
                if(position == "center"){
                    aprilTagPosition = "right";
                }
            }
            if(aprilTagPosition == ""){
                cameraServo.setPosition(0.65);
                sleep(800);
                position = camera.getDesiredAprilTagPosition(aprilTagId);
                if(position == "center"){
                    aprilTagPosition = "center";
                }
            }
            if(aprilTagPosition == ""){
                cameraServo.setPosition(0.55);
                sleep(800);
                position = camera.getDesiredAprilTagPosition(aprilTagId);
                if(position == "center"){
                    aprilTagPosition = "left";
                }
            }
            if(aprilTagPosition != ""){
                telemetry.addData("Apriltag found", aprilTagPosition);
                telemetry.update();
            }else{
                telemetry.addData("No Apriltag found", "bombing left side");
                aprilTagPosition = "left";
                telemetry.update();
            }

            /**TIRAR PATO Y DEJAR PIXEL**/
            chassis.goToDistanceX(-70,1);
            telemetry.addData("complete", "yes");
            armDown();
//            if(position == "center"){
//                //center
//                //place pixel
//                openClaw();
//                aprilTagId = 5;
//            }else if(position == "right"){
//                //right
//                chassis.turnDegrees(-50, 0.5);
//                openClaw();
//                aprilTagId = 6;
//            }else{
//                //left
//                chassis.turnDegrees(50, 0.5);
//                openClaw();
//                aprilTagId = 4;
//            }
//
//            //TODO: capaz y tiene que regresar a orientación 0 e ir un poco atrás para pasar el puente
//
//            /**DEJAR PIXEL EN BACKSTAGE **/
//            chassis.turnDegrees(90, 0.5);
//            chassis.goToDistance(-50,70,0.5);
//            timePassed = time.seconds();
//            while ((position != "center") || (time.seconds() < timePassed + 5)) {
//                if(position != "center"){
//                    position = camera.getDesiredAprilTagPosition(aprilTagId);
//                    chassis.leftRun(0.5);
//                    chassis.postCurrentPosition();
//                    telemetry.update();
//                }else{
//                    chassis.stopChassis();
//                    chassis.postCurrentPosition();
//                }
//                if(time.seconds() < timePassed + 5) {
//                    jackUp(1);
//                if(time.seconds() < timePassed + 2){
//                    drawSlidersOut(0.5);
//                }
//                }else{
//                    stopJack();
//                    stopSlidersOut();
//                }
//            }





//            chassis.goToDistance(-35, 70, 0.5);
//            chassis.goToDistance(-35, -10, 0.5); //TODO: a ver si esto puede ir más rápido

            /**TOMAR PIXEL**/


            /**ESTACIONAR**/


           sleep(30000);
        }
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
}
