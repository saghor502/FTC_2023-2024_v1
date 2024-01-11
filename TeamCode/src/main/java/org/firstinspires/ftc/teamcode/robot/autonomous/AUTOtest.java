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
public class AUTOtest extends LinearOpMode {
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
            chassis.goToYDistance(1, 0.5);
            chassis.rotateDirection(90, 0.5);
            sleep(30000);
        }
    }
}
