package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.RobotInit;
import org.firstinspires.ftc.teamcode.robot.init.cameras.AprilTagOpenCvCamera;
import org.firstinspires.ftc.teamcode.robot.init.cameras.ColorAverageOpenCvCamera;

@TeleOp(name="April Camera Test", group="3 Testers")
public class TLOPAprilTagCameraCalibration extends LinearOpMode {

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        AprilTagOpenCvCamera aprilCamera = new AprilTagOpenCvCamera(hardwareMap, telemetry, "Webcam 2");

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
//            if(gamepad1.dpad_left){
//                robot.cameraLeft();
//            }else if(gamepad1.dpad_up){
//                robot.cameraMiddle();
//            }else if(gamepad1.dpad_right){
//                robot.cameraRight();
//            }else{
//                robot.storeCamera();
//            }

            aprilCamera.telemetryAprilTag();
        }
        aprilCamera.stopCameraStream();
    }
}
