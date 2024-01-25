package org.firstinspires.ftc.teamcode.robot.init;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Position;

import java.lang.reflect.Method;
import java.util.function.Function;

public class Chassis_v2 {
    private DcMotorEx rightFront, rightRear, leftFront, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private IMU imu;
    private VoltageSensor batteryVoltageSensor;
    private static Telemetry telemetry;

    private Position currentPosition = new Position(0, 0, 0);

    private double ticksPerInches = 331.4583333;

    private double maxdistanceError = 1;
    private double maxorientationError = 2;
    double initialhead = 0;

    int startingLEpos;
    int startingREpos;
    int strartingHEpos;

    public Chassis_v2(HardwareMap hardwareMap, Telemetry telemetry) {
        Chassis_v2.telemetry = telemetry;

        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rb"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lb"));

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();

        startingLEpos = leftEncoder.getCurrentPosition();
        startingREpos = rightEncoder.getCurrentPosition();
        strartingHEpos = frontEncoder.getCurrentPosition();
    }

    public void initChassis(){
        initialhead =  (int) Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**TESTER FUNCTIONS*/
    public Position getPosition() {
        return currentPosition;
    }

    /**POSITIONS**/
//    public void updatePosition(int hEncoder, int lEncoder, int rEncoder){
//
//        double x = currentPosition.getXPosition();
//        double y = currentPosition.getYPosition();
//        int theta = currentPosition.getOrientation();
//
//        int Error;
//
//        if(lEncoder != -rEncoder){
//            currentPosition.setXPosition();
//        }else{
//
//        }
//
//        currentPosition.setYPosition();
//        currentPosition.setOrientation();
//
//        telemetry.addData("x", currentPosition.getXPosition());
//        telemetry.addData("y", currentPosition.getYPosition());
//        telemetry.addData("theta", currentPosition.getOrientation());
//    }

    /**MOVEMENT**/
    public void goToPosition(Position position){

    }

    public void move(double forwardPower, double leftPower, double turnPower){
        //ly = forward, lx = leftRun, rx = turnRight
        double leftY, leftX, rightX;

        //TODO: if the robot can handle turns right, then erase turn error
        double fbrferror = 0, fbrrerror = 0, fblferror = 0, fblrerror = 0;
        double lrrferror = 0, lrrrerror = 0, lrlferror = 0, lrlrerror = 0;
        //double turnrferror = 0, turnrrerror = 0, turnlferror = 0, turnlrerror = 0;
        double rferror = 0, rrerror = 0, lferror = 0, lrerror = 0;

        int fbdirection, lrdirection, turndirection;

        int deltalEpos, deltarEpos, deltahEpos;

        int lEpos = startingLEpos - leftEncoder.getCurrentPosition();
        int rEpos = startingREpos- rightEncoder.getCurrentPosition();
        int hEpos = strartingHEpos - frontEncoder.getCurrentPosition();

        //updatePosition(hEpos, lEpos, rEpos);

        //MAKE THE VARIABLES ZERO IN CASE OF ERROR
        if(forwardPower < 0.2 && forwardPower > -0.2){
            leftY = 0;
            fbdirection = 0;
        }else{
            leftY = forwardPower;
            if(leftY>0){
                fbdirection = 1;
            }else{
                fbdirection = -1;
            }
        }
        if(leftPower < 0.2 && leftPower > -0.2){
            leftX = 0;
            lrdirection = 0;
        }else{
            leftX = leftPower;
            if(leftX>0){
                lrdirection = 1;
            }else{
                lrdirection = -1;
            }
        }
        if(turnPower < 0.2 && turnPower > -0.2){
            rightX = 0;
            turndirection = 0;
        }else{
            rightX = turnPower;
            if(rightX>0){
                turndirection = 1;
            }else{
                turndirection = -1;
            }
        }

        //TODO: if the variable can't change, make a sleep function

        deltahEpos = frontEncoder.getCurrentPosition() - hEpos;
        deltalEpos = leftEncoder.getCurrentPosition() - lEpos;
        deltarEpos = rightEncoder.getCurrentPosition() - rEpos;

        if((fbdirection != 0) && (deltahEpos != 0)) {
            //is turning while its going forward
            if (deltahEpos > 0) { //TODO: if it keeps "fixing" itself unnecessarily, then make an error for the deltahEpos
                //TODO: if it goes to the other side, change the variables from right to left and the other way around
                if(deltahEpos > 5000){
                    fbrferror = -1;
                    fbrrerror = -1;
                }else{
                    fbrferror = (-(float) deltahEpos/5000);
                    fbrrerror = (-(float) deltahEpos/5000);
                }
            } else {
                if(deltahEpos < -5000){
                    fblferror = 1;
                    fblrerror = 1;
                }else{
                    fblferror = ((float) deltahEpos/5000);
                    fblrerror = ((float) deltahEpos/5000);
                }
            }
        }else{
            fbrferror = 0;
            fbrrerror = 0;
            fblferror = 0;
            fblrerror = 0;
        }

        if((lrdirection != 0) && ((deltalEpos != 0)||(deltarEpos != 0))) {
            //is turning while its going forward
            if (deltahEpos > 0) { //TODO: if it keeps "fixing" itself unnecessarily, then make an error for the deltahEpos
                //TODO: if it goes to the other side, change the variables from right to left and the other way around
                if(deltahEpos > 5000){
                    lrrferror = -1;
                    lrrrerror = -1;
                }else{
                    lrrferror = (-(float) deltahEpos/5000);
                    lrrrerror = (-(float) deltahEpos/5000);
                }
            } else {
                if(deltahEpos < -5000){
                    lrlferror = 1;
                    lrlrerror = 1;
                }else{
                    lrlferror = ((float) deltahEpos/5000);
                    lrlrerror = ((float) deltahEpos/5000);
                }
            }
        }else{
            lrrferror = 0;
            lrrrerror = 0;
            lrlferror = 0;
            lrlrerror = 0;
        }

        rferror = (fbrferror + lrrferror)/2;
        rrerror = (fbrrerror + lrrrerror)/2;
        lferror = (fblferror + lrlferror)/2;
        lrerror = (fblrerror + lrlrerror)/2;

        if(leftX != 0 && leftY != 0 && rightX != 0){
            //all of them are moving
            rightFront.setPower((((leftY - leftX - rightX)/3) + rferror)/2);
            rightRear.setPower((((leftY + leftX - rightX)/3) + rrerror)/2);
            leftFront.setPower((((-leftY + leftX - rightX)/3) + lferror)/2);
            leftRear.setPower((((-leftY - leftX - rightX)/3) + lrerror)/2);

        }else if((leftX != 0  && leftY != 0 && rightX == 0)
                || (leftX != 0  && leftY == 0 && rightX != 0)
                || (leftX == 0  && leftY != 0 && rightX != 0)){
            //two are true
            rightFront.setPower((((leftY - leftX - rightX)/2) + rferror)/2);
            rightRear.setPower((((leftY + leftX - rightX)/2) + rrerror)/2);
            leftFront.setPower((((-leftY + leftX - rightX)/2) + lferror)/2);
            leftRear.setPower((((-leftY - leftX - rightX)/2) + lrerror)/2);

        }else if((leftX != 0  && leftY == 0 && rightX == 0)
                || (leftX == 0  && leftY != 0 && rightX == 0)
                || (leftX == 0  && leftY == 0 && rightX != 0)){
            //one is true
            rightFront.setPower(((leftY - leftX - rightX) + rferror)/2);
            rightRear.setPower(((leftY + leftX - rightX) + rrerror)/2);
            leftFront.setPower(((-leftY + leftX - rightX) + lferror)/2);
            leftRear.setPower(((-leftY - leftX - rightX) + lrerror)/2);

        }else{
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
        }

        telemetry.addData("horizontal Pos", hEpos);
        telemetry.addData("right Pos", rEpos);
        telemetry.addData("left Pos", lEpos);
    }
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
