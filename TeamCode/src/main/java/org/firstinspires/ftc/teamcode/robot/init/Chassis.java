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

    private double distanceError = 5;
    private double orientationError = 2;
    double initialhead = 0;

    int numberOfEncoders = 0;

    public Chassis(DcMotorEx rF, DcMotorEx rR, DcMotorEx lF, DcMotorEx lR,
                   Encoder lE, Encoder rE, Encoder fE, HardwareMap hardwareMap, Telemetry telemetry) {
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
        initialhead =  this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**TESTER FUNCTIONS*/
    public Position getPosition() {
        return currentPosition;
    }

    public Position updatePosition() {
        //save position and orientation
        currentPosition.setOrientation((int) Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialhead));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = (leftEncoder.getCurrentPosition() / ticksPerInches) * Math.cos(-botHeading) - (frontEncoder.getCurrentPosition() / ticksPerInches) * Math.sin(-botHeading);
        double rotY = (frontEncoder.getCurrentPosition() / ticksPerInches) * Math.sin(-botHeading) + (leftEncoder.getCurrentPosition() / ticksPerInches) * Math.cos(-botHeading);

        currentPosition.setXPosition(rotX);
        currentPosition.setYPosition(rotY);

        return currentPosition;
    }

    public Position newUpdateOnPos(double gamepadX, double gamepadY, double gamepadRX){
        currentPosition.setOrientation((int) Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialhead));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation

        gamepadY = -gamepadY;

        double rotX = gamepadX * Math.cos(-botHeading) - gamepadY * Math.sin(-botHeading);
        double rotY = gamepadX * Math.sin(-botHeading) + gamepadY * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(gamepadRX), 1);

        double lf = rotY + rotX + gamepadRX / denominator; //Le pones al motor el poder de este, ej. leftFack.setPower(lf);
        double lb = rotY - rotX + gamepadRX / denominator;
        double rf = rotY - rotX - gamepadRX / denominator;
        double rb = rotY + rotX - gamepadRX / denominator;

        currentPosition.setXPosition(rotX);
        currentPosition.setYPosition(rotY);

        return currentPosition;
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
            this.currentPosition.setOrientation(Math.toDegrees(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
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

    /*synchronous method*/
    public void goToPosition(Position position, double robotPower) {
        double distanceToX = currentPosition.getXPosition() - position.getXPosition();
        double distanceToY = currentPosition.getYPosition() - position.getYPosition();

        while((currentPosition.getOrientation() != position.getOrientation()) &&
                (Math.abs(distanceToX) <= Math.abs(position.getXPosition()) + distanceError) &&
                (Math.abs(distanceToY) <= Math.abs(position.getYPosition()) + distanceError)){
            //save position and orientation
            this.currentPosition.setOrientation(Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            this.currentPosition.setXPosition(frontEncoder.getCurrentPosition() * (Math.sin(Math.toRadians(currentPosition.getOrientation()))));
            if(numberOfEncoders == 3){
                this.currentPosition.setYPosition((leftEncoder.getCurrentPosition() / ticksPerInches) * (Math.cos(Math.toRadians(currentPosition.getOrientation()))));
            }else if(numberOfEncoders == 2){
                double encoderAverage =  (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition())/2;
                this.currentPosition.setYPosition(encoderAverage * encoderAverage * (Math.cos(Math.toRadians(currentPosition.getOrientation()))));
            }else{
                break;
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
