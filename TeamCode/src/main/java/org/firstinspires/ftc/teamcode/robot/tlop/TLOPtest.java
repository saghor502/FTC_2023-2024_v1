package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous
public class TLOPtest extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor leftRear;

    @Override
    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");
        waitForStart();

        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear);

        while (!isStopRequested()){
            if((gamepad1.right_stick_y > 0.2)||(gamepad1.right_stick_y < -0.2)){
                chassis.forward(gamepad1.right_stick_y);
            }

            if((gamepad1.left_stick_x > 0.2)||(gamepad1.left_stick_x < -0.2)){
                chassis.turnRight(gamepad1.left_stick_x);
            }

            if((gamepad1.right_stick_x > 0.2)||(gamepad1.right_stick_x < -0.2)){
                chassis.leftRun(gamepad1.right_stick_x);
            }
        }
    }
}
