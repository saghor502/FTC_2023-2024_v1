package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOPPositionCalibration extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder,
                hardwareMap, telemetry);
        chassis.initChassis();

        waitForStart();
        while (!isStopRequested()){

            if((gamepad1.left_stick_y > 0.2) || (gamepad1.left_stick_y < -0.2) ||
                    (gamepad1.right_stick_x > 0.2) || (gamepad1.right_stick_x < -0.2) ||
                    (gamepad1.left_stick_x > 0.2) || (gamepad1.left_stick_x < -0.2)){

                if ((gamepad1.left_stick_y > 0.2) || (gamepad1.left_stick_y < -0.2)) {
                    chassis.forward(gamepad1.left_stick_y);
                }
                if ((gamepad1.right_stick_x > 0.2) || (gamepad1.right_stick_x < -0.2)) {
                    chassis.turnRight(gamepad1.right_stick_x);
                }
                if ((gamepad1.left_stick_x > 0.2) || (gamepad1.left_stick_x < -0.2)) {
                    chassis.leftRun(gamepad1.left_stick_x);
                }
            }else{
                chassis.stopChassis();
            }

            double x = Math.round(chassis.updatePosition().getXPosition());
            double y = Math.round(chassis.updatePosition().getYPosition());
            int theta = (int) chassis.updatePosition().getOrientation();

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("orientation", theta);
            telemetry.update();
        }
    }
}
