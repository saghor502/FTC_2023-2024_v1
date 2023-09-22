package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.autonomous.database.PosTestDB;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous
public class AUTOtest extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    @Override
    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");
        leftEncoder = hardwareMap.get(DcMotor.class, "lb");
        rightEncoder = hardwareMap.get(DcMotor.class, "lb");
        frontEncoder = hardwareMap.get(DcMotor.class, "lb");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder, imu);
        while (!isStopRequested()){
            PosTestDB DB = new PosTestDB();
            chassis.runToPosition(DB.position1);
            chassis.turnToDegree(DB.position1);
        }
    }
}
