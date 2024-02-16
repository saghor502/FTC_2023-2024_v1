package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@TeleOp(name="Encoder Test", group="3 Tester")
public class TLOPEncoderTest extends LinearOpMode {
    private Servo cameraServo;

    @Override
    public void runOpMode(){
        cameraServo = hardwareMap.get(Servo.class, "cameraS");
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        waitForStart();
        while (!isStopRequested()){
            if(gamepad1.right_trigger > 0.2){
                chassis.move(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
            }else{
                chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            telemetry.addData("front Encoder", chassis.getFrontEncoderPos());
            telemetry.addData("right Encoder", chassis.getRightEncoderPos());
            telemetry.addData("left Encoder", chassis.getLeftEncoderPos());
            telemetry.update();
        }
    }
}
