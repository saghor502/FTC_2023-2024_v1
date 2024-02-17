package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.RobotInit;

@Autonomous(name="Safe Mode", group="1 Auto")
public class SafeMode  extends LinearOpMode {

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        RobotInit robot = new RobotInit(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            chassis.forward(0.3);
            sleep(2700);
            chassis.stopChassis();
            robot.armUp();
            sleep(2000);
            robot.openClaw();
            sleep(1000);
            chassis.forward(-0.1);
            sleep(2500);
            chassis.stopChassis();
            robot.closeClaw();
            sleep(1000);
            robot.armDown();

            chassis.stopChassis();

            sleep(30000);
        }
    }
}
