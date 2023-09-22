package org.firstinspires.ftc.teamcode.robot.init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odometry.position.Position;

public class Chassis {
    private static DcMotor rightFront, rightRear, leftFront, leftRear;
    private static DcMotor leftEncoder, rightEncoder, frontEncoder;
    private static BNO055IMU imu;

    private double inchesError = 5;
    private double orientationError = 2;

    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                   DcMotor lE, DcMotor rE, DcMotor fE, BNO055IMU imu) {

    }
    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR) {

    }

    public Chassis(DcMotor rF, DcMotor rR, DcMotor lF, DcMotor lR,
                   DcMotor lE, DcMotor fE, BNO055IMU imu) {

    }

    /**ENCODER/IMU RUNNING**/
    public void runToPosition(Position position) {
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
        rightFront.setPower(-power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        leftRear.setPower(-power);
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
