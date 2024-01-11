package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

@Autonomous
public class ParkBlueClose extends LinearOpMode {
    private DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private DcMotor slidervert;


    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear,
                leftEncoder, rightEncoder, frontEncoder,
                hardwareMap, telemetry);
        chassis.initChassis();
        slidervert = hardwareMap.get(DcMotor.class, "sV");

        //TODO: init

        waitForStart();
        while (!isStopRequested()) {
            /**Anotaciones**/
            //chassis.rotate(90, 1);
            //chassis.goToYDistance(-48, 1);
            //chassis.goToPosition(new Position(48, 50, 90), 1);

            //motor poder 1
            slidervert.setPower(1);
            //INSTR
            sleep(2000);
            slidervert.setPower(0);
            sleep(4000);
            //motor poder 0


            //inicio de autonomo
            chassis.goToYDistance(-48, 0.8);
            chassis.goToXDistance(6, 0.8);
            chassis.goToYDistance(-24, 0.7);
            chassis.goToXDistance(-18, 0.8);
            chassis.goToYDistance(3, 0.7);
            chassis.goToXDistance(-24, 0.7);
            chassis.goToYDistance(12, 0.8);
            //garra
            chassis.goToYDistance(72, 0.7);
            chassis.goToXDistance(12, 0.6);
            //coloca pixel
            chassis.goToXDistance(-12, 0.6);
            chassis.goToYDistance(-72, 0.7);
            //garra
            chassis.goToYDistance(72, 0.7);
            chassis.goToXDistance(12, 0.6);
            //colocapixel
            chassis.goToXDistance(-12, 0.6);
            chassis.goToYDistance(24, 0.6);

            sleep(30000);
        }
    }
}
