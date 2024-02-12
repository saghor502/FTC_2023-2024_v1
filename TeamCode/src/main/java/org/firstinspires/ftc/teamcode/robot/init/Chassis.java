package org.firstinspires.ftc.teamcode.robot.init;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.odometry.position.Position;

import java.lang.reflect.Method;
import java.util.function.Function;

public class Chassis {
    private DcMotor rightFront, rightRear, leftFront, leftRear;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private IMU imu;
    private static Telemetry telemetry;

    private Position currentPosition = new Position(0, 0, 0);

    private final double ticksPerInches = 238.99371049052830188679245283018;

    private double maxdistanceError = 0.5;
    private double maxorientationError = 1;
    double initialhead = 0;

    double initialXPosition = 0;
    double initialYPosition = 0;

    int startingLEpos;
    int startingREpos;
    int strartingHEpos;
    double startingHead;

    org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition globalPositionUpdate = null;

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        Chassis.telemetry = telemetry;

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder = hardwareMap.get(DcMotor.class, "rb");
        rightEncoder = hardwareMap.get(DcMotor.class, "rf");
        frontEncoder = hardwareMap.get(DcMotor.class, "lb");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();

        startingLEpos = leftEncoder.getCurrentPosition();
        startingREpos = rightEncoder.getCurrentPosition();
        strartingHEpos = frontEncoder.getCurrentPosition();
        startingHead = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        globalPositionUpdate = new org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition(leftEncoder, rightEncoder, frontEncoder, ticksPerInches, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        initialXPosition = globalPositionUpdate.returnXCoordinate();
        initialYPosition = globalPositionUpdate.returnYCoordinate();
    }

    public void goToPosition (double targetXPosition, double targetYPosition, double robotPower){
        double desiredRobotOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        postCurrentPosition();
        double xPosition = currentPosition.getXPosition();
        double yPosition = currentPosition.getYPosition();
        double head = currentPosition.getOrientation();
        double initialHead = currentPosition.getOrientation();

        double turnPower = 0;

        double distanceToXTarget = targetXPosition - xPosition;
        double distanceToYTarget = targetYPosition - yPosition;

        double pivotCorrection = desiredRobotOrientation - head;
        double pivotspeed;
        double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));


        while ((distanceToXTarget > maxdistanceError) || (distanceToYTarget > maxdistanceError)) {

            distanceToXTarget = targetXPosition - xPosition;
            distanceToYTarget = targetYPosition - yPosition;
//            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

//            robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotmovementxcomponent = calculateX(robotMovementAngle, robotPower, desiredRobotOrientation);
            double robotmovementycomponent = calculateY(robotMovementAngle, robotPower, desiredRobotOrientation);

            pivotCorrection = - desiredRobotOrientation + head;
            pivotspeed = robotPower * 0.05;

            if(head > initialHead){
                turnPower = 0.3;
            }else if(head < initialHead){
                turnPower = -0.3;
            }else{
                turnPower = 0;
            }
            //MOTORES
            leftFront.setPower(robotmovementycomponent + robotmovementxcomponent + (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed) - turnPower);
            rightFront.setPower(robotmovementycomponent - robotmovementxcomponent - (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed)  + turnPower);
            leftRear.setPower(robotmovementycomponent - robotmovementxcomponent + (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed) - turnPower);
            rightRear.setPower(robotmovementycomponent + robotmovementxcomponent - (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed) + turnPower);

            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

            postCurrentPosition();
            yPosition = currentPosition.getYPosition();
            xPosition = currentPosition.getXPosition();
            head = currentPosition.getOrientation();

            telemetry.addData("leftFront Power", robotmovementycomponent + robotmovementxcomponent + (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed));
            telemetry.addData("rightFront Power", robotmovementycomponent - robotmovementxcomponent - (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed));
            telemetry.addData("leftRear Power", robotmovementycomponent - robotmovementxcomponent + (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed));
            telemetry.addData("rightRear Power", robotmovementycomponent + robotmovementxcomponent - (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed));
        }

        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        if((int) currentPosition.getOrientation() != (int) desiredRobotOrientation) {
            //double orientationDistance = desiredRobotOrientation - currentPosition.getOrientation();
            turnDegrees((int) desiredRobotOrientation, 0.5);
        }
        stopChassis();
    }

    public void turnDegrees(int degrees, double power){
        int currentDegrees = currentPosition.getOrientation();
        int degreesToTheta = currentDegrees + degrees;
        int orientationSign = 0;

        if(degreesToTheta == 0){
            orientationSign = 0;
        }else{
            orientationSign = -(int) degreesToTheta/Math.abs((int) degreesToTheta);
        }

        while(Math.abs(degreesToTheta) > maxorientationError){

            currentDegrees = currentPosition.getOrientation();
            degreesToTheta = currentDegrees + degrees;
            if(!(degreesToTheta == 0)) {
                orientationSign = (int) degreesToTheta / Math.abs((int) degreesToTheta);
            }

            if(Math.abs(degreesToTheta) > maxorientationError + 15){
                move(0,0,power * orientationSign);
            }else if((Math.abs(degreesToTheta) <= maxorientationError + 15) && (Math.abs(degreesToTheta) > maxorientationError)){
                move(0,0,0.2 * orientationSign);
            }else{
                move(0,0,0);
            }
            postCurrentOrientation();
            telemetry.addData("power", power * orientationSign);
            telemetry.addData("degrees to theta", degreesToTheta);
            telemetry.addData("", "");
        }
        postCurrentOrientation();
        stopChassis();
    }

    /**POST CURRENT POSITIONS**/
    public void postCurrentPosition(){
        double leposition = leftEncoder.getCurrentPosition() - startingLEpos;
        double reposition = rightEncoder.getCurrentPosition() - startingREpos;
        double heposition = frontEncoder.getCurrentPosition() - strartingHEpos;

        double fbaverage = (leposition - reposition)/2;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
        double absBotHeading = Math.abs(botHeading);

        //TODO: cancelar los errores de los encoders mientras el robot gira
        if((absBotHeading >= 0) && (absBotHeading < 180)){
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
            currentPosition.setXPosition(-globalPositionUpdate.returnYCoordinate() / ticksPerInches);
            currentPosition.setYPosition(-globalPositionUpdate.returnXCoordinate() / ticksPerInches);
        }else{
            int cnt = 0;
            double range = Math.sqrt(absBotHeading);

            for (int i = 2; i <= range; i++) {
                if (absBotHeading % i == 0) {
                    int j = i;
                    while (absBotHeading % j == 0 && absBotHeading != 0) {
                        absBotHeading = absBotHeading / j;
                        j *= i;
                        cnt++;
                    }
                    while (absBotHeading % i == 0) {
                        absBotHeading /= i;
                    }
                }
            }
            startingHead = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 180 * cnt;
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        globalPositionUpdate.stop();

        currentPosition.setOrientation((int) botHeading);
        telemetry.addData("x", currentPosition.getXPosition());
        telemetry.addData("y", currentPosition.getYPosition());
        telemetry.addData("orientation", currentPosition.getOrientation());
        telemetry.update();
    }
    public void postCurrentOrientation(){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
        double absBotHeading = Math.abs(botHeading);

        //TODO: cancelar los errores de los encoders mientras el robot gira <- NTH
        if(absBotHeading == 0 || absBotHeading < 180){
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
        }else if(absBotHeading > 180){
            int cnt = 0;
            double range = Math.sqrt(absBotHeading);

            for (int i = 2; i <= range; i++) {
                if (absBotHeading % i == 0) {
                    int j = i;
                    while (absBotHeading % j == 0 && absBotHeading != 0) {
                        absBotHeading = absBotHeading / j;
                        j *= i;
                        cnt++;
                    }
                    while (absBotHeading % i == 0) {
                        absBotHeading /= i;
                    }
                }
            }
            startingHead = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (180 * cnt);
            botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        currentPosition.setOrientation((int) botHeading);
        telemetry.addData("x", currentPosition.getXPosition());
        telemetry.addData("y", currentPosition.getYPosition());
        telemetry.addData("orientation", currentPosition.getOrientation());
        telemetry.update();
    }

    public double calculateX(double desiredAngle, double speed, double desiredRobotOrientation) {
        if (desiredRobotOrientation > 89 && desiredRobotOrientation < 91)
            return -(Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if ((desiredRobotOrientation > -1 && desiredRobotOrientation < 1) || (desiredRobotOrientation > -359 && desiredRobotOrientation < 361))
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 179 && desiredRobotOrientation < 181 )
            return -(Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > -91 && desiredRobotOrientation < -89 )
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation < -179 && desiredRobotOrientation > -181 )
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 1 && desiredRobotOrientation < 89)
            return (Math.sin(Math.toRadians(desiredAngle)) * speed) - (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
    }
    private double calculateY(double desiredAngle, double speed, double desiredRobotOrientation) {
        if (desiredRobotOrientation > 89 && desiredRobotOrientation < 91)
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if ((desiredRobotOrientation > -1 && desiredRobotOrientation < 1)|| (desiredRobotOrientation > -359 && desiredRobotOrientation < 361))
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 179 && desiredRobotOrientation < 181)
            return -(Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > -91 && desiredRobotOrientation < -89)
            return -(Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation < -179 && desiredRobotOrientation > -181)
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 1 && desiredRobotOrientation < 89)
            return (Math.cos(Math.toRadians(desiredAngle)) * speed) + (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
    }

    /**POWER RUNNING**/
    public void move(double forwardPower, double leftPower, double turnPower){
        //ly = forward, lx = leftRun, rx = turnRight
        telemetry.addData("status", "moving");
        double leftY;
        double leftX;
        double rightX;

        //MAKE THE VARIABLES ZERO IN CASE OF ERROR
        if(forwardPower < 0.2 && forwardPower > -0.2){
            leftY = 0;
        }else{
            leftY = forwardPower;
        }
        if(leftPower < 0.2 && leftPower > -0.2){
            leftX = 0;
        }else{
            leftX = leftPower;
        }
        if(turnPower < 0.2 && turnPower > -0.2){
            rightX = 0;
        }else{
            rightX = turnPower;
        }

        if(leftX != 0 && leftY != 0 && rightX != 0){
            rightFront.setPower((leftY - leftX - rightX)/3);
            rightRear.setPower((leftY + leftX - rightX)/3);
            leftFront.setPower((-leftY + leftX - rightX)/3);
            leftRear.setPower((-leftY - leftX - rightX)/3);
        }else if((leftX != 0  && leftY != 0 && rightX == 0)
                || (leftX != 0  && leftY == 0 && rightX != 0)
                || (leftX == 0  && leftY != 0 && rightX != 0)){
            rightFront.setPower((leftY - leftX - rightX)/2);
            rightRear.setPower((leftY + leftX - rightX)/2);
            leftFront.setPower((-leftY + leftX - rightX)/2);
            leftRear.setPower((-leftY - leftX - rightX)/2);
        }else if((leftX != 0  && leftY == 0 && rightX == 0)
                || (leftX == 0  && leftY != 0 && rightX == 0)
                || (leftX == 0  && leftY == 0 && rightX != 0)){
            rightFront.setPower(leftY - leftX - rightX);
            rightRear.setPower(leftY + leftX - rightX);
            leftFront.setPower(-leftY + leftX - rightX);
            leftRear.setPower(-leftY - leftX - rightX);
        }else{
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
        }
    }
    public void forward(double power){
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        leftFront.setPower(power);
        leftRear.setPower(power);
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
