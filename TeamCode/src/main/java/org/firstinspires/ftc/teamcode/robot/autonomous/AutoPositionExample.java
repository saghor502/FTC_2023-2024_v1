package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;


@Autonomous
public class AutoPositionExample  extends LinearOpMode {

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested()){
            while(opModeIsActive()){
                chassis.goToDistance(60, 0, 1);
                sleep(1000);
                chassis.goToDegrees(90, 0.5);
                sleep(1000);
                telemetry.addData("x", chassis.getCurrentPosition().getXPosition());
                telemetry.addData("y", chassis.getCurrentPosition().getYPosition());
                telemetry.addData("theta", chassis.getCurrentPosition().getOrientation());
                telemetry.update();
                //chassis.turnDegrees(90, 0.5);

                sleep(30000);
            }
        }
    }
}
