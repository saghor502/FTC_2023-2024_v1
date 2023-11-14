package org.firstinspires.ftc.teamcode.robot.init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.position.Position;

public class Chassis {
    private static DcMotor rightFront, rightRear, leftFront, leftRear;
    private static DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;
    private static Telemetry telemetry;

    private double inchesError = 5;
    private double orientationError = 2;

    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                   DcMotor lE, DcMotor rE, DcMotor fE,
                   BNO055IMU imu, HardwareMap hardwareMap, Telemetry telemetry) {
        Chassis.rightFront = rF;
        Chassis.rightRear = rR;
        Chassis.leftFront = lF;
        Chassis.leftRear  = lR;

        Chassis.leftEncoder = lE;
        Chassis.rightEncoder = rE;
        Chassis.frontEncoder = fE;

        Chassis.telemetry = telemetry;

        Chassis.imu = imu;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");

        leftEncoder = hardwareMap.get(DcMotor.class, "lE");
        rightEncoder = hardwareMap.get(DcMotor.class, "rE");
        frontEncoder = hardwareMap.get(DcMotor.class, "fE");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                    DcMotor lE, DcMotor fE,
                    BNO055IMU imu, HardwareMap hardwareMap, Telemetry telemetry) {
        Chassis.rightFront = rF;
        Chassis.rightRear = rR;
        Chassis.leftFront = lF;
        Chassis.leftRear  = lR;

        Chassis.leftEncoder = lE;
        Chassis.frontEncoder = fE;

        Chassis.telemetry = telemetry;

        Chassis.imu = imu;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");

        leftEncoder = hardwareMap.get(DcMotor.class, "lE");
        frontEncoder = hardwareMap.get(DcMotor.class, "fE");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                   BNO055IMU bno055IMU, HardwareMap hardwareMap, Telemetry telemetry) {
        Chassis.rightFront = rF;
        Chassis.rightRear = rR;
        Chassis.leftFront = lF;
        Chassis.leftRear  = lR;

        Chassis.telemetry = telemetry;

        Chassis.imu = bno055IMU;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    /**ENCODER/IMU RUNNING**/
    public void goToPosition(Position position) {
    }
    public void turnToDegree(Position position) {
        double curdeg=Math.toDegrees(imu.getAngularOrientation().firstAngle)-orientationError;
        double pivot = (orientationError - curdeg);

        double initialhead=Math.toDegrees(imu.getAngularOrientation().firstAngle);

        while(Math.abs(pivot) > Math.abs(imu.getAngularOrientation().firstAngle + orientationError)){
            double heading = imu.getAngularOrientation().firstAngle;

            if (heading>121&&heading<360){
                heading=heading-360;
            }
            if (heading<-151&&heading>-360){
                heading=heading+360;
            }

            double pivotCorrection = position.getOrientation() - heading;

            if (pivotCorrection > 20 || (pivotCorrection < -180)) {
                turnRight(.6);

            } else if (pivotCorrection < -20 && (pivotCorrection > -180)) {
                turnRight(-.6);

            } else if (pivotCorrection < 20 && (pivotCorrection > 0)) {
                turnRight(.2);

            } else if (pivotCorrection > -20 && (pivotCorrection < 0)) {
                turnRight(-.2);

            }else{
                stopChassis();
            }
            pivot=pivotCorrection;


            telemetry.addData("Desired Orientation: ", position.getOrientation());
            telemetry.addData("Current Orientation", pivot);
            telemetry.addData("Current Orientation", pivotCorrection);
            telemetry.addData("Current Orientation", heading);
            telemetry.update();

            stopChassis();
        }

    }

    /**POWER RUNNING**/
    public void forward(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
    }
    public void leftRun(double power){
        rightFront.setPower(-power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        leftRear.setPower(-power);
    }
    public void turnRight(double power){
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
    }
    public void stopChassis(){
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
    }
}
