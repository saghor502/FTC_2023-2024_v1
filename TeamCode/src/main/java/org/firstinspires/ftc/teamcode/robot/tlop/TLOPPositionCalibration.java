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
            if(gamepad1.right_trigger > 0.2){
                chassis.move(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
            }else{
                chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            chassis.updatePosition();

            double x = Math.round(chassis.getPosition().getXPosition());
            double y = Math.round(chassis.getPosition().getYPosition());
            int theta = chassis.getPosition().getOrientation();

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("orientation", theta);
            telemetry.update();
        }
    }
}
