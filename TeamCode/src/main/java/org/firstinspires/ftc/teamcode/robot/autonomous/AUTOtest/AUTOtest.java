package org.firstinspires.ftc.teamcode.robot.autonomous.AUTOtest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.domparser.DomParser;
import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.firstinspires.ftc.teamcode.robot.autonomous.AUTOtest.database.PosTestDB;
import org.firstinspires.ftc.teamcode.robot.init.Chassis;

import javax.xml.parsers.ParserConfigurationException;

@Autonomous
public class AUTOtest extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, imu, hardwareMap, telemetry);
        DomParser DB = new DomParser("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/autonomous/AUTOtest/database/AUTOtest.xml");
        PosTestDB DB2 = new PosTestDB();

        Position newPosition = new Position(1,1,90);

        waitForStart();
        while (!isStopRequested()){
            try {
                chassis.turnToDegree(DB2.position1);


                DB.setPosition("1", newPosition);
            } catch (ParserConfigurationException e) {
                e.printStackTrace();
                telemetry.addData("Found Error while reading data from DataBse", e);
                telemetry.update();
            }
        }
    }
}
