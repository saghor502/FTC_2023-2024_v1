package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOPtest extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, imu, hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested()){
            if((gamepad1.left_stick_y > 0.2)||(gamepad1.left_stick_y < -0.2)){
                chassis.forward(gamepad1.left_stick_y);
            }

            if((gamepad1.right_stick_x > 0.2)||(gamepad1.right_stick_x < -0.2)){
                chassis.turnRight(gamepad1.right_stick_x);
            }

            if((gamepad1.left_stick_x > 0.2)||(gamepad1.left_stick_x < -0.2)){
                chassis.leftRun(gamepad1.left_stick_x);
            }
            chassis.stopChassis();
        }
    }
}
