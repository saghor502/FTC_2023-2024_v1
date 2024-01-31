package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.OpenCvCamera;

@TeleOp
public class TLOPCalibration extends LinearOpMode {

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        OpenCvCamera camera = new OpenCvCamera(hardwareMap, telemetry);
        sleep(1500);
        telemetry.addData("Open Cv Camera", "ready to roll");
        telemetry.update();

        waitForStart();
        while (!isStopRequested()){
            if(gamepad1.right_trigger > 0.2){
                chassis.move(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
            }else{
                chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if(gamepad1.right_trigger > 0.2){
                telemetry.speak("Bwomp");
            }
            telemetry.addData("Orientation", chassis.getCurrentPosition().getOrientation());
            telemetry.addData("x", chassis.getCurrentPosition().getXPosition());
            telemetry.addData("y", chassis.getCurrentPosition().getYPosition());
            camera.telemetryAprilTag();
            telemetry.update();
        }
    }
}
