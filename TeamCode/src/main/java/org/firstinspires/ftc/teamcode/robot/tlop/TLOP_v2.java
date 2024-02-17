package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.init.Chassis;
import org.firstinspires.ftc.teamcode.robot.init.RobotInit;

@TeleOp(name="TLOP", group="1 TLOP")
public class TLOP_v2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        RobotInit robot = new RobotInit(hardwareMap, telemetry);

        /**Initial setup**/
        robot.boxMiddle();

        waitForStart();

        while (!isStopRequested()) {

            /**CHASSIS**/
            chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /**INTAKE**/
            if(gamepad1.right_bumper){
//                //abre claw
                robot.closeClaw();
            }else if(gamepad1.left_bumper){
                //cierra claw
//                claw.setPosition(0.5);
                robot.openClaw();
            }
            if(gamepad1.right_trigger > 0.2){
                //sube arm
                robot.armUp();
                robot.closeClaw();
            }else if(gamepad1.left_trigger > 0.2){
                //baja arm
                robot.armDown();
                robot.closeClaw();
            }

            /**OUTAKE**/
            if((gamepad2.right_stick_y > 0.2) || (gamepad2.right_stick_y < -0.2)){
                if(gamepad2.right_stick_y > 0.2){
                    robot.drawSlidersOut(gamepad2.right_stick_y);
                }else if(gamepad2.right_stick_y < -0.2){
                    robot.drawSlidersOut(gamepad2.right_stick_y);
                }
            }else{
                robot.stopSlidersOut();
            }
            if(gamepad2.right_trigger > 0.2){
                robot.boxDown();
            }else if(gamepad2.right_bumper){
                robot.boxUp();
            }else{
                robot.boxMiddle();
            }
            if(gamepad2.left_trigger > 0.2){
                robot.outOpen();
            }else{
                robot.outClose();
            }
            if(gamepad2.dpad_up){
                //jack goes up
                robot.jackUp(1);
            }else if(gamepad2.dpad_down){
                //jack goes down
                robot.jackDown(1);
            }else{
                robot.stopJack();
            }


            /**MISC**/
            if((gamepad2.left_stick_y > 0.2) || (gamepad2.left_stick_y < -0.2)){
                if(gamepad2.left_stick_y > 0.2){
                    robot.hangSlidersOut(gamepad2.left_stick_y);
                }else if(gamepad2.left_stick_y < -0.2){
                    robot.hangSlidersOut(gamepad2.left_stick_y);
                }
            }else{
                robot.stopHangSliders();
            }
        }
    }
}
