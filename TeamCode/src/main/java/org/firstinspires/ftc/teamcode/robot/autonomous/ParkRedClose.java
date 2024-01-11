package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous
public class ParkRedClose extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;


    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder,
                hardwareMap, telemetry);
        chassis.initChassis();

        waitForStart();
        while (!isStopRequested()){
            chassis.rotate(90, 0.4);
            chassis.goToYDistance(-48, 0.4);
            //chassis.rotate(90, 0.6);
            telemetry.addData("Status", "Complete!");
            telemetry.update();
            sleep(30000);
        }
    }
}
