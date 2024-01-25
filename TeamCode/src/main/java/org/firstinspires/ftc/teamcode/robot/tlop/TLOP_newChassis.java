package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis_v2;

@TeleOp
public class TLOP_newChassis extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis_v2 chassis = new Chassis_v2(hardwareMap, telemetry);
        chassis.initChassis();

        waitForStart();

        while (!isStopRequested()) {
            /**CHASSIS**/
            chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
