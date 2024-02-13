package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous(name="1px|Backdrop + park") 
public class BackDropPlace  extends LinearOpMode {

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            chassis.goToPosition(0, -28, 0.5);
            sleep(1000);
            chassis.goToPosition(28, -28, 0.5);
            sleep(1000);
            armDown();
            sleep(1000);
            openClaw();
            sleep(1000);
            chassis.goToPosition(0, -28, 0.5);
            sleep(1000);
            closeClaw();
            sleep(1000);
            armUp();
            sleep(1000);
            chassis.goToPosition(28, 0, 0.5);
            sleep(1000);
            chassis.goToPosition(56, 0, 0.5);
            sleep(1000);
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
}
