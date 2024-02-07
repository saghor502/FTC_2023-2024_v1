package org.firstinspires.ftc.teamcode.robot.tlop;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.robot.init.Chassis;
//import org.firstinspires.ftc.teamcode.robot.init.AprilTagOpenCvCamera;
//import org.firstinspires.ftc.teamcode.robot.init.ColorAverageOpenCvCamera;
//
//@TeleOp
//public class TLOPCalibration extends LinearOpMode {
//    private Servo cameraServo;
//
//    @Override
//    public void runOpMode(){
//        cameraServo = hardwareMap.get(Servo.class, "cameraS");
//        Chassis chassis = new Chassis(hardwareMap, telemetry);
//
//        AprilTagOpenCvCamera aprilCamera = new AprilTagOpenCvCamera(hardwareMap, telemetry);
//        ColorAverageOpenCvCamera colorCamera = new ColorAverageOpenCvCamera(hardwareMap);
//        telemetry.addData("Open Cv Camera", "ready to roll");
//        telemetry.update();
//
//        waitForStart();
//        while (!isStopRequested()){
//            if(gamepad1.right_trigger > 0.2){
//                chassis.move(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
//            }else{
//                chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            }
//            if(gamepad1.right_trigger > 0.2){
//                telemetry.speak("Bwomp");
//            }
//            if(gamepad1.dpad_left){
//                cameraServo.setPosition(0.55);
//            }else if(gamepad1.dpad_up){
//                cameraServo.setPosition(0.65);
//            }else if(gamepad1.dpad_right){
//                cameraServo.setPosition(0.75);
//            }else{
//                cameraServo.setPosition(1);
//            }
//            telemetry.addData("Orientation", chassis.getCurrentPosition().getOrientation());
//            telemetry.addData("x", chassis.getCurrentPosition().getXPosition());
//            telemetry.addData("y", chassis.getCurrentPosition().getYPosition());
//            aprilCamera.telemetryAprilTag();
//            colorCamera.setAlliance("red");
//            int element_zone = colorCamera.elementDetection(telemetry);
//        }
//    }
//}
