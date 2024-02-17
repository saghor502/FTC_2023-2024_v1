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
            chassis.goToXDistance(5, 0.5);
        }
    }
}
