package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous(name="Position Example", group="3 Examples")
public class AutoPositionExample  extends LinearOpMode {

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            chassis.goToPosition(60, 0, 0.8);
            sleep(1000);
            chassis.turnDegrees(-90, 0.5);
            sleep(1000);

            sleep(30000);
        }
    }
}
