package org.firstinspires.ftc.teamcode.robot.autonomous.AUTOtest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Disabled
@Autonomous
public class AUTOtest extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private Servo angel;


    public void runOpMode(){
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
        Chassis roberto = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder,
                hardwareMap, telemetry);
        roberto.initChassis();
        angel =  hardwareMap.get(Servo.class, "claw");

        waitForStart();
        while (!isStopRequested()){
//            chassis.turnToDegree(new Position(1, 2, 180), claw.getPosition() != 1, new Runnable() {
//                @Override
//                public void run() {
//                    claw.setPosition(1);
//                }
//            });
//            chassis.turnToDegree(new Position(1, 2, 0));
//            roberto.goToPosition(new Position(1, 1, 1), -0.5);
//            roberto.forward(1);
//            sleep(500);
//            roberto.stopChassis();
//
//            sleep(30000);

            roberto.updatePosition();
            telemetry.addData("x", roberto.getPosition().getXPosition());
            telemetry.addData("y", roberto.getPosition().getYPosition());
            telemetry.addData("orientation", roberto.getPosition().getOrientation());
            telemetry.update();
        }
    }

    public void sayHello(){
        System.out.println("Hello, World!");
    }
}
