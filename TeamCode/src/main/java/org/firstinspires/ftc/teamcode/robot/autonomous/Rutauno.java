

package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous(name="Heading test", group="3 Examples") //Me acabo de acordar q si agarramos de ahi agarramos mas pixels del limite pero ya me tengo que ir asi q ya nimodo 
public class Rutauno  extends LinearOpMode {

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            chassis.goToPosition(0, 35, 0.8);
            sleep(1000);
            chassis.goToPosition(0, 28, 0.2);
            sleep(1000)
            chassis.goToPosition(-60, 28, 0.8);
            sleep(1000);
            chassis.turnDegrees(180, 0.5);
            sleep(1000);
            armDown();
            sleep(1000);
            openClaw();
            sleep(1000);
            closeClaw();
            sleep(1000);
            armUp();
            sleep(1000);
            chassis.goToPosition(0, 28, 0.8);
            sleep(2000);
            chassis.goToPosition(0, 0, 0.2);
            sleep(1000);
            chassis.goToPosition(56, 0, 0.2);
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
