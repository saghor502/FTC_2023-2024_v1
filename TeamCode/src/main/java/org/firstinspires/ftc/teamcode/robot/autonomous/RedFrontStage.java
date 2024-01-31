package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.OpenCvCamera;


@Autonomous
public class RedFrontStage  extends LinearOpMode {

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested()){
            while(opModeIsActive()){
               chassis.goToDistance(0,10,0.5);
               //camara
               chassis.goToDegrees(180,0.5);
               //garra
               //garrax2
               chassis.goToDegrees(90,0.5);
               chassis.goToDistance(19,0,0.5);
               chassis.goToDistance(0,25,0.5);
               //garra
               //garrax2
               chassis.goToDistance(-108,0,0.5);
               chassis.goToDistance(0,-12, 0.5);
               chassis.goToDistance(0,13,0.5);
               //camaras
               //garraycajamucho=)
               chassis.goToDistance()




                sleep(30000);
            }
        }
    }
}
