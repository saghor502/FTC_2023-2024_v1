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
import org.json.JSONException;

import javax.xml.parsers.ParserConfigurationException;

//Hola
// Adios
@Autonomous
public class AUTOtest extends LinearOpMode {
    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(rightFront, rightRear, leftFront, leftRear, imu, hardwareMap, telemetry);
        DomParser DB = new DomParser("AUTOtest.json");
        PosTestDB DB2 = new PosTestDB();

        waitForStart();
        while (!isStopRequested()){
            try {
                telemetry.addData("xPosition: ", DomParser.writeData());
            } catch (JSONException e) {
                e.printStackTrace();
                telemetry.addData("xPosition: ", e.toString());
            }
            telemetry.update();
        }
    }
}
