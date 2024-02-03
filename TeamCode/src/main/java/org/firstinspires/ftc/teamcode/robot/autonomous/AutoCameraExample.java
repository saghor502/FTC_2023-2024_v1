package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.AprilTagOpenCvCamera;

@Autonomous
public class AutoCameraExample extends LinearOpMode {
    String position = "";

    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        AprilTagOpenCvCamera camera = new AprilTagOpenCvCamera(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            //chassis.goToPosition(new Position(5, 0, 0), 0.5);
            while (position != "center") {
                position = camera.getDesiredAprilTagPosition(6);
                chassis.leftRun(0.5);
                telemetry.update();
            }
            chassis.leftRun(0);
            telemetry.addData("Position found: ", position);
            telemetry.update();
            sleep(30000);

        }
    }
}
