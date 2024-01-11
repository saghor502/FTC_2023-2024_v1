package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp
public class TLOP_v2 extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Servo brazo1, brazo2;
    private CRServo cat;
    private static BNO055IMU imu;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, hardwareMap, telemetry);
        chassis.initChassis();

        cat =  hardwareMap.get(CRServo.class, "cat");
        brazo1 =  hardwareMap.get(Servo.class, "brazo1");
        brazo2 =  hardwareMap.get(Servo.class, "brazo2");

        waitForStart();

        while (!isStopRequested()) {

            /**CHASSIS**/
            chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /**INTAKE**/
            if(gamepad1.right_bumper){
                brazo1.setPosition(1);
                brazo2.setPosition(0);
            }else if(gamepad1.left_bumper){
                brazo1.setPosition(0);
                brazo2.setPosition(1);
            }

            /**OUTAKE*/
            if(gamepad2.dpad_up){
                cat.setPower(1);
            }else if(gamepad2.dpad_down){
                cat.setPower(-1);
            }else{
                cat.setPower(0);
            }

            telemetry.update();
        }
    }
}
