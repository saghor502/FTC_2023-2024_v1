package org.firstinspires.ftc.teamcode.robot.autonomous.AutoMove;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous
public class AutoMove extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;


    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder,
                hardwareMap, telemetry);
        chassis.initChassis();

        waitForStart();
        while (!isStopRequested()){
            chassis.goToPosition(new Position(1, 0, 0), 0.2);
            telemetry.update();
        }
    }
}
