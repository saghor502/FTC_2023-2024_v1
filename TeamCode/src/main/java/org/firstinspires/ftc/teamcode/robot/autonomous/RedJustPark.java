//package org.firstinspires.ftc.teamcode.robot.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robot.init.Chassis;
//import org.firstinspires.ftc.teamcode.robot.init.RobotInit;
//import org.firstinspires.ftc.teamcode.robot.init.cameras.AprilTagOpenCvCamera;
//import org.firstinspires.ftc.teamcode.robot.init.cameras.ColorAverageOpenCvCamera;
//
//
//@Autonomous(name="Red Just Park", group="1 Auto Red")
//public class RedJustPark  extends LinearOpMode {
//    private Servo cameraServo;
//
//    //AUTONOMOUS VARIABLES
//    String aprilPosition = "";
//    double timePassed = 0;
//
//    public void runOpMode(){
//        Chassis chassis = new Chassis(hardwareMap, telemetry);
//        RobotInit robot = new RobotInit(hardwareMap, telemetry);
//        AprilTagOpenCvCamera aprilCamera = new AprilTagOpenCvCamera(hardwareMap, telemetry);
//        ColorAverageOpenCvCamera colorCamera = new ColorAverageOpenCvCamera(hardwareMap);
//        ElapsedTime time = new ElapsedTime();
//
//        cameraServo = hardwareMap.get(Servo.class, "cameraS");
//
//
//        colorCamera.setAlliance("red");
//        int element_zone = colorCamera.elementDetection(telemetry);
//
//        telemetry.update();
//
//        robot.closeClaw();
//        robot.boxUp();
//        robot.outClose();
//
//        time.startTime();
//
//        waitForStart();
//
//        while (!isStopRequested() && opModeIsActive()) {
//            time.reset();
//
//            chassis.goToDistance(0,10,0.5);
//
//            /**ESTACIONAR**/
//            chassis.goToDistance(50,10,0.5);
//            chassis.stopChassis();
//
//            sleep(30000);
//        }
//    }
//
//
//}
