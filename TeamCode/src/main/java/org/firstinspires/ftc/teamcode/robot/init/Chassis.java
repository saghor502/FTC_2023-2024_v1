package org.firstinspires.ftc.teamcode.robot.init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.odometry.position.Position;

public class Chassis {
    private static DcMotor rightFront, rightRear, leftFront, leftRear;
    private static DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    private double inchesError = 5;
    private double orientationError = 2;

    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                   DcMotor lE, DcMotor rE, DcMotor fE,
                   BNO055IMU imu, HardwareMap hardwareMap) {
        Chassis.rightFront = rF;
        Chassis.rightRear = rR;
        Chassis.leftFront = lF;
        Chassis.leftRear  = lR;

        Chassis.leftEncoder = lE;
        Chassis.rightEncoder = rE;
        Chassis.frontEncoder = fE;

        Chassis.imu = imu;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");

        leftEncoder = hardwareMap.get(DcMotor.class, "lE");
        rightEncoder = hardwareMap.get(DcMotor.class, "rE");
        frontEncoder = hardwareMap.get(DcMotor.class, "fE");
    }
    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                    DcMotor lE, DcMotor fE,
                    BNO055IMU imu, HardwareMap hardwareMap) {
        Chassis.rightFront = rF;
        Chassis.rightRear = rR;
        Chassis.leftFront = lF;
        Chassis.leftRear  = lR;

        Chassis.leftEncoder = lE;
        Chassis.frontEncoder = fE;

        Chassis.imu = imu;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");

        leftEncoder = hardwareMap.get(DcMotor.class, "lE");
        frontEncoder = hardwareMap.get(DcMotor.class, "fE");
    }
    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                   BNO055IMU imu, HardwareMap hardwareMap) {
        Chassis.rightFront = rF;
        Chassis.rightRear = rR;
        Chassis.leftFront = lF;
        Chassis.leftRear  = lR;

        Chassis.imu = imu;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");
    }

    /**ENCODER/IMU RUNNING**/
    public void goToPosition(Position position) {
    }
    public void turnToDegree(Position position) {
        while(!(imu.getAngularOrientation().firstAngle >= position.getOrientation() + orientationError)||
                !(imu.getAngularOrientation().firstAngle <= position.getOrientation() - orientationError)){
            if (!(imu.getAngularOrientation().firstAngle >= position.getOrientation() + orientationError + 10)||
                    !(imu.getAngularOrientation().firstAngle <= position.getOrientation() - orientationError + 10)){
                turnRight(1);
            }else if(!(imu.getAngularOrientation().firstAngle >= position.getOrientation() + (orientationError + 5)||
                    !(imu.getAngularOrientation().firstAngle <= position.getOrientation() - (orientationError + 5)))){
                turnRight(0.5);
            }

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
        rightFront.setPower(power);
        rightRear.setPower(-power);
        leftFront.setPower(-power);
        leftRear.setPower(power);
    }
    public void turnRight(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        leftRear.setPower(power);
    }
    public void stopChassis(){
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
    }
}
