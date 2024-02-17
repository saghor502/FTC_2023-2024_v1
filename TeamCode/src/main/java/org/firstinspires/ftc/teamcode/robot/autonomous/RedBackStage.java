package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.RobotInit;
import org.firstinspires.ftc.teamcode.robot.init.cameras.AprilTagOpenCvCamera;
import org.firstinspires.ftc.teamcode.robot.init.cameras.ColorAverageOpenCvCamera;

@Autonomous(name="Red Back Stage", group="1 Auto")
public class RedBackStage extends LinearOpMode {

    String position = "";
    int aprilId = 0;
    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        RobotInit robot = new RobotInit(hardwareMap, telemetry);
        AprilTagOpenCvCamera camera = new AprilTagOpenCvCamera(hardwareMap, telemetry, "Webcam 2");
        ColorAverageOpenCvCamera colorCamera = new ColorAverageOpenCvCamera(hardwareMap, "Webcam 1");

        //busca el pato
        colorCamera.setAlliance("red");
        int element_zone = colorCamera.elementDetection(telemetry);
        sleep(1000);
        if(element_zone == 1){
            aprilId = 5;
        }else if(element_zone == 2){
            aprilId = 6;
        }else{
            aprilId = 4;
        }

        robot.boxMiddle();
        robot.closeClaw();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            chassis.goToXDistance(50, 0.5);
            chassis.turnDegrees(180, 0.5);

            //deja el pixel
            if(element_zone == 1){
                robot.armDown();
                sleep(500);
                robot.openClaw();
                sleep(500);
                robot.closeClaw();
                robot.armUp();
            }else if(element_zone == 2){
                chassis.turnDegrees(30, 0.5);
                robot.armDown();
                sleep(500);
                robot.openClaw();
                sleep(500);
                robot.closeClaw();
                robot.armUp();
            }else{
                chassis.turnDegrees(-30, 0.5);
                robot.armDown();
                sleep(500);
                robot.openClaw();
                sleep(500);
                robot.closeClaw();
                robot.armUp();
            }
            sleep(1000);

            chassis.turnDegrees(-90, 0.5);
            chassis.goToXDistance(-30, 0.3);
            chassis.forward(-0.5);
            sleep(800);
            chassis.stopChassis();
            sleep(500);
            while (position != "center") {
                position = camera.getDesiredAprilTagPosition(aprilId);
                chassis.leftRun(-0.5);
                telemetry.update();
            }
            chassis.stopChassis();
            //deja
            robot.jackUp(1);
            sleep(1000);
            robot.stopJack();
            robot.drawSlidersOut(0.5);
            sleep(1000);
            robot.stopSlidersOut();
            robot.boxDown();
            sleep(500);
            robot.outOpen();
            sleep(1000);
            robot.outClose();
            robot.boxMiddle();
            sleep(1000);
            robot.insertSlidersOut(0.5);
            sleep(1000);
            robot.stopSlidersOut();
            robot.jackDown(1);
            sleep(1000);
            robot.stopJack();

            //estaciona
            chassis.leftRun(-0.3);
            sleep(2000);
            chassis.stopChassis();
            sleep(250);
            chassis.forward(0.3);
            sleep(1000);
            chassis.stopChassis();

            //final
            sleep(30000);
        }
    }
}
