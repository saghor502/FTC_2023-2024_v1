package org.firstinspires.ftc.teamcode.robot.init;

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

public class Chassis {
    private DcMotorEx rightFront, rightRear, leftFront, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private IMU imu;
    private VoltageSensor batteryVoltageSensor;
    private static Telemetry telemetry;

    private Position currentPosition = new Position(0, 0, 0);

    private double ticksPerInches = 331.4583333;

    private double distanceError = 1;
    private double orientationError = 2;
    double initialhead = 0;

    int numberOfEncoders = 0;

    public Chassis(DcMotorEx rF, DcMotorEx rR, DcMotorEx lF, DcMotorEx lR,
                   Encoder lE, Encoder rE, Encoder fE,
                   HardwareMap hardwareMap, Telemetry telemetry) {
        numberOfEncoders = 2;
        rightFront = rF;
        rightRear = rR;
        leftFront = lF;
        leftRear = lR;

        leftEncoder = lE;
        rightEncoder = rE;
        frontEncoder = fE;

        Chassis.telemetry = telemetry;

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
    }

    public Chassis(DcMotorEx rF, DcMotorEx rR, DcMotorEx lF, DcMotorEx lR,
                   Encoder lE, Encoder fE, HardwareMap hardwareMap, Telemetry telemetry) {
        numberOfEncoders = 2;
        rightFront = rF;
        rightRear = rR;
        leftFront = lF;
        leftRear = lR;

        leftEncoder = lE;
        frontEncoder = fE;

        Chassis.telemetry = telemetry;

        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rb"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lb"));

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public Chassis(DcMotorEx rF, DcMotorEx rR, DcMotorEx lF, DcMotorEx lR, HardwareMap hardwareMap, Telemetry telemetry) {
        numberOfEncoders = 0;
        rightFront = rF;
        rightRear = rR;
        leftFront = lF;
        leftRear = lR;

        Chassis.telemetry = telemetry;

        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void initChassis(){
        initialhead =  (int) Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**TESTER FUNCTIONS*/
    public Position getPosition() {
        return currentPosition;
    }

    public void updatePosition() {
        //save position and orientation
        currentPosition.setOrientation((int) Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialhead));
        double botHeading = currentPosition.getOrientation();

        // Rotate the movement direction counter to the bot's rotation
        //este no esta chido
        double rotX = (frontEncoder.getCurrentPosition() / ticksPerInches) * Math.cos(-botHeading) - (leftEncoder.getCurrentPosition() / ticksPerInches) * Math.sin(-botHeading);
        //este esta chido
        double rotY = (frontEncoder.getCurrentPosition() / ticksPerInches) * Math.sin(-botHeading) + (leftEncoder.getCurrentPosition() / ticksPerInches) * Math.cos(-botHeading);

        currentPosition.setXPosition(rotX);
        currentPosition.setYPosition(rotY);
    }


    /**POSITION MOVEMENT**/
    /*asynchronous method*/
    public void goToPosition(Position position, double robotPower, boolean clause, Runnable func) {
        double distanceToX = currentPosition.getXPosition() - position.getXPosition();
        double distanceToY = currentPosition.getYPosition() - position.getYPosition();

        double totalDistance = Math.hypot(distanceToX, distanceToY);

        while((currentPosition.getOrientation() != position.getOrientation()) &&
                (Math.abs(distanceToX) <= Math.abs(position.getXPosition()) + distanceError) &&
                (Math.abs(distanceToY) <= Math.abs(position.getYPosition()) + distanceError) &&
                clause){
            //save position and orientation
            this.currentPosition.setOrientation( (int) Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            this.currentPosition.setXPosition(frontEncoder.getCurrentPosition() * (Math.sin(Math.toRadians(currentPosition.getOrientation()))));
            if(numberOfEncoders == 3){
                this.currentPosition.setYPosition((leftEncoder.getCurrentPosition() / ticksPerInches) * (Math.cos(Math.toRadians(currentPosition.getOrientation()))));
                func.run();
            }else if(numberOfEncoders == 2){
                double encoderAverage =  (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition())/2;
                this.currentPosition.setYPosition(encoderAverage / ticksPerInches * (Math.cos(Math.toRadians(currentPosition.getOrientation()))));
                func.run();
            }else{
                if(clause){
                    func.run();
                }else{
                    break;
                }
            }

            //set powers to all motors
            double robotmovementycomponent = calculateX(this.currentPosition.getOrientation(), robotPower, position.getOrientation());
            double robotmovementxcomponent = calculateY(this.currentPosition.getOrientation(), robotPower, position.getOrientation());

            double pivotCorrection = position.getOrientation() - currentPosition.getOrientation();
            double pivotspeed = .05 * robotPower;

            this.leftFront.setPower(-(robotmovementycomponent - robotmovementxcomponent + (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed)));
            this.rightFront.setPower((robotmovementycomponent + robotmovementxcomponent - (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed)));
            this.leftRear.setPower(-(robotmovementycomponent + robotmovementxcomponent + (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed)));
            this.rightRear.setPower((robotmovementycomponent - robotmovementxcomponent - (Math.sin(Math.toRadians(pivotCorrection)) * pivotspeed)));
        }
    }
    public void goToYDistance(double Y, double power){
        double currentY = currentPosition.getYPosition();
        double distanceSlow = distanceError + 0.5;
        double desiredY = currentY + Y;
        double direction = Y/Math.abs(Y);

        forward(0.5);

        while ((Math.abs(currentY) > Math.abs(desiredY + distanceError)) || (Math.abs(currentY) < Math.abs(desiredY - distanceError))){
            currentY = currentPosition.getYPosition();

            move(0, power * direction, 0);

            updatePosition();

            telemetry.addData("current y", currentY);
            telemetry.addData("Position to go", desiredY);
            telemetry.update();
        }

        stopChassis();
    }

    public void goToXDistance(double X, double power){
        double currentY = currentPosition.getYPosition();
        double distanceSlow = distanceError + 0.5;
        double desiredY = currentY + X;
        double direction = X/Math.abs(X);

        forward(0.5);

        while ((Math.abs(currentY) > Math.abs(desiredY + distanceError)) || (Math.abs(currentY) < Math.abs(desiredY - distanceError))){
            currentY = currentPosition.getYPosition();

            forward(power * direction);

            updatePosition();

            telemetry.addData("current y", currentY);
            telemetry.addData("Position to go", desiredY);
            telemetry.update();
        }

        stopChassis();
    }

    public void rotate(double Degree, double power){
        double currentTheta = currentPosition.getOrientation();
        double desiredOrientation = currentTheta + Degree;
        double direction = (Degree - currentTheta)/Math.abs(Degree - currentTheta);

        turnRight(0.5);

        while ((Math.abs(currentTheta) < Math.abs(desiredOrientation + orientationError)) && (Math.abs(currentTheta) > Math.abs(desiredOrientation - orientationError))){
            currentTheta = currentPosition.getOrientation();

            turnRight(power * direction);

            updatePosition();

            telemetry.addData("current Theta", currentTheta);
            telemetry.addData("Orientation to go", desiredOrientation);
            telemetry.update();
        }

        stopChassis();
    }

    public void rotateDirection(double angle, double power) {
        double leftEncoderDistance = leftEncoder.getCurrentPosition() / ticksPerInches;
        double rightEncoderDistance = rightEncoder.getCurrentPosition() / ticksPerInches;
        double currentOrientation = ((leftEncoderDistance - rightEncoderDistance) / (2 * 7.58));
        double radianAngle = Math.toRadians(angle);
        double powerQuirality = (radianAngle / Math.abs(radianAngle));
        double Cpower = (powerQuirality * Math.abs(power));
        while (Math.abs(radianAngle) > Math.abs(currentOrientation)){
            turnRight(Cpower);
            telemetry.addData("power xd", Cpower);
            telemetry.addData("orientation", currentOrientation);
            telemetry.addData("radian", radianAngle);
            telemetry.update();
        }
        stopChassis();
    }

    /*synchronous method*/
    public void goToPosition(Position position, double robotPower){
        double currentX = currentPosition.getXPosition();
        double currentY = currentPosition.getYPosition();
        double currentTheta = currentPosition.getOrientation();

        double distanceSlowed = distanceError + 0.5;

        double powerLY = 0.3, powerLX = 0.3, powerRX = 0.3;

        while(((currentX > position.getXPosition() + distanceError) || (currentX < position.getXPosition() - distanceError))
            || ((currentY > position.getYPosition() + distanceError) || (currentY < position.getYPosition() - distanceError))
            || ((currentTheta > position.getOrientation() + orientationError) || (currentTheta < position.getOrientation() - orientationError))){
            currentX = currentPosition.getXPosition();
            currentY = currentPosition.getYPosition();
            currentTheta = currentPosition.getOrientation();

            double distanceToX = Math.abs(currentPosition.getXPosition()) - Math.abs(position.getXPosition());
            double distanceToY = Math.abs(currentPosition.getYPosition()) - Math.abs(position.getYPosition());
            double distanceToTheta = Math.abs(currentPosition.getOrientation()) - Math.abs(position.getOrientation());

            if(distanceToX > distanceSlowed){

                powerLY = robotPower;
            }else if((distanceToX < distanceSlowed)&&(distanceToX > distanceError)){
                powerLY = 0.2;
            }else{
                powerLY = 0;
            }

            if(distanceToY > distanceSlowed){
                powerLX = robotPower;
            }else if((distanceToY < distanceSlowed)&&(distanceToY > distanceError)){
                powerLX = 0.2;
            }else{
                powerLY = 0;
            }

            if(distanceToTheta > distanceSlowed){
                powerRX = robotPower;
            }else if((distanceToY < distanceSlowed)&&(distanceToY > distanceError)){
                powerRX = 0.2;
            }else{
                powerRX = 0;
            }

            move(powerLY, powerLX, powerRX);

            telemetry.addData("x", currentX);
            telemetry.addData("y", currentY);
            telemetry.addData("theta", currentTheta);
            telemetry.update();

            updatePosition();
        }
    }

    /**IMU TURNS**/
    /*asynchronous method*/
    public void turnToDegree(Position position, boolean clause, Runnable func) {
        double curdeg = Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double pivot = (position.getOrientation() - curdeg);

        while ((Math.abs(pivot) > orientationError) && (clause)) {
            double heading = Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            if (heading > 121 && heading < 360) {
                heading = heading - 360;
            }
            if (heading < -151 && heading > -360) {
                heading = heading + 360;
            }
            double pivotCorrection = position.getOrientation() - heading;

            if (pivotCorrection > 20 || (pivotCorrection < -180)) {
                this.turnRight(0.6);

            } else if (pivotCorrection < -20 && (pivotCorrection > -180)) {
                this.turnRight(-0.6);

            } else if (pivotCorrection < 20 && (pivotCorrection > 0)) {
                this.turnRight(0.2);

            } else if (pivotCorrection > -20 && (pivotCorrection < 0)) {
                this.turnRight(-0.2);
            }else{
                this.stopChassis();
            }
            pivot = pivotCorrection;

            func.run();
        }
    }

    /*synchronous method*/
    public void turnToDegree(Position position) {
        double curdeg = Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double pivot = (position.getOrientation() - curdeg);

        while (Math.abs(pivot) > orientationError) {
            double heading = Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            if (heading > 121 && heading < 360) {
                heading = heading - 360;
            }
            if (heading < -151 && heading > -360) {
                heading = heading + 360;
            }
            double pivotCorrection = position.getOrientation() - heading;

            if (pivotCorrection > 20 || (pivotCorrection < -180)) {
                this.turnRight(0.6);

            } else if (pivotCorrection < -20 && (pivotCorrection > -180)) {
                this.turnRight(-0.6);

            } else if (pivotCorrection < 20 && (pivotCorrection > 0)) {
                this.turnRight(0.2);

            } else if (pivotCorrection > -20 && (pivotCorrection < 0)) {
                this.turnRight(-0.2);
            }else{
                this.stopChassis();
            }
            pivot = pivotCorrection;
        }
    }

    /**POWER RUNNING**/
    public void move(double forwardPower, double leftPower, double turnPower){
        //ly = forward, lx = leftRun, rx = turnRight
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

    /**MATH SHENANIGANS**/
    public double calculateX(double desiredAngle, double speed, double desiredRobotOrientation) {
        if (desiredRobotOrientation > 89 && desiredRobotOrientation < 91)
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if ((desiredRobotOrientation > -1 && desiredRobotOrientation < 1) || (desiredRobotOrientation > 359 && desiredRobotOrientation < 361))
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 179 && desiredRobotOrientation < 181 )
            return -(Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > -95 && desiredRobotOrientation < -85)
            return -(Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 265 && desiredRobotOrientation < 275)
            return -(Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation < -179 && desiredRobotOrientation > -181 )
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 1 && desiredRobotOrientation < 89)
            return (Math.sin(Math.toRadians(desiredAngle)) * speed) - (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
    }
    private double calculateY(double desiredAngle, double speed, double desiredRobotOrientation) {
        if (desiredRobotOrientation > 89 && desiredRobotOrientation < 91)
            return -(Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if ((desiredRobotOrientation > -1 && desiredRobotOrientation < 1)|| (desiredRobotOrientation > 359 && desiredRobotOrientation < 361))
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 179 && desiredRobotOrientation < 181)
            return -(Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > -95 && desiredRobotOrientation < -85 )
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 265 && desiredRobotOrientation < 275)
            return (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation < -179 && desiredRobotOrientation > -181)
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
        else if (desiredRobotOrientation > 1 && desiredRobotOrientation < 89)
            return (Math.cos(Math.toRadians(desiredAngle)) * speed) + (Math.sin(Math.toRadians(desiredAngle)) * speed);
        else
            return (Math.cos(Math.toRadians(desiredAngle)) * speed);
    }
}
