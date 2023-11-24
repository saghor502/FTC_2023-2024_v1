package org.firstinspires.ftc.teamcode.robot.autonomous.AUTOtest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous
public class AUTOtest extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private Servo angel;

    @Override
    public void runOpMode(){
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        Chassis roberto = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder,
                imu, hardwareMap, telemetry);
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

            telemetry.addData("x", roberto.testerGetPosition().getXPosition());
            telemetry.addData("y", roberto.testerGetPosition().getYPosition());
            telemetry.addData("orientation", roberto.testerGetPosition().getOrientation());
            telemetry.update();
        }
    }

    public void sayHello(){
        System.out.println("Hello, World!");
    }
}
