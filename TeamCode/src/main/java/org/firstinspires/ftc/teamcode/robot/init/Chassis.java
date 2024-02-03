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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.odometry.Encoder;
import org.firstinspires.ftc.teamcode.odometry.position.Position;

import java.lang.reflect.Method;
import java.util.function.Function;

public class Chassis {
    private DcMotorEx rightFront, rightRear, leftFront, leftRear;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private IMU imu;
    private static Telemetry telemetry;

    private Position currentPosition = new Position(0, 0, 0);

    private double ticksPerInches = 298.74213811316037735849056603773;

    private double maxdistanceError = 2;
    private double maxorientationError = 1;
    double initialhead = 0;

    int startingLEpos;
    int startingREpos;
    int strartingHEpos;
    int startingHead;

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
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

        startingLEpos = leftEncoder.getCurrentPosition();
        startingREpos = rightEncoder.getCurrentPosition();
        strartingHEpos = frontEncoder.getCurrentPosition();
        startingHead = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public Position getCurrentPosition(){
        postCurrentPosition();
        return(currentPosition);
    }

    /**POSITIONS**/
   public void postCurrentPosition(){
       double leposition = leftEncoder.getCurrentPosition() - startingLEpos;
       double reposition = rightEncoder.getCurrentPosition() - startingREpos;
       double heposition = frontEncoder.getCurrentPosition() - strartingHEpos;

       double fbaverage = (leposition - reposition)/2;

       int botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
       int absBotHeading = Math.abs(botHeading);

       //TODO: cancelar los errores de los encoders mientras el robot gira
        if((absBotHeading >= 0) && (absBotHeading < 180)){
            botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
            currentPosition.setXPosition((Math.ceil((fbaverage/ticksPerInches) * Math.cos(absBotHeading)) * 100)/100);
            currentPosition.setYPosition((Math.ceil((heposition/ticksPerInches) * Math.sin(absBotHeading)) * 100)/100);
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
            botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

       telemetry.addData("x", currentPosition.getXPosition());
       telemetry.addData("y", currentPosition.getYPosition());
       telemetry.addData("orientation", currentPosition.getOrientation());
       telemetry.update();
       currentPosition.setOrientation(botHeading);
   }
   public void postCurrentOrientation(){
       int botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
       double absBotHeading = Math.abs(botHeading);

       //TODO: cancelar los errores de los encoders mientras el robot gira <- NTH
       if(absBotHeading == 0 || absBotHeading < 360){
           botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startingHead;
       }else if(absBotHeading > 360){
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
           startingHead = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 360 * cnt;
           botHeading = (int) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
       }

       currentPosition.setOrientation(botHeading);
       telemetry.addData("x", currentPosition.getXPosition());
       telemetry.addData("y", currentPosition.getYPosition());
       telemetry.addData("orientation", currentPosition.getOrientation());
       telemetry.update();
   }

   public void goToDistance(double x, double y, double power){
       double currentX = currentPosition.getXPosition();
       double currentY = currentPosition.getYPosition();
       int originalOrientation = currentPosition.getOrientation();

       double distanceToX = currentX - x;
       double distanceToY = currentY - y;
       double compensateDegrees = 0;

       double fbpower = 0;
       double rlpower = 0;
       double compensatePower = 0;

       int signX = (int) (distanceToX/Math.abs(distanceToX));
       int signY = (int) (distanceToY/Math.abs(distanceToY));
       int orientationSign = 0;

       forward(0.5);

       while((Math.abs(distanceToX) > maxdistanceError) || (Math.abs(distanceToY) > maxdistanceError)){
           postCurrentPosition();
           currentX = currentPosition.getXPosition();
           currentY = currentPosition.getYPosition();

           distanceToX = currentX - x;
           distanceToY = currentY - y;

           signX = (int) (distanceToX/Math.abs(distanceToX));
           signY = -(int) (distanceToY/Math.abs(distanceToY));

           if(currentPosition.getOrientation() != originalOrientation){
               if(Math.abs(compensateDegrees) > 5){
                   compensatePower = 0.3 * orientationSign;
               }else{
                   compensatePower = 0;
               }
           }
           if(currentPosition.getOrientation() != 0){
               orientationSign = currentPosition.getOrientation() == 0 ? 0 :
                       currentPosition.getOrientation()/ Math.abs(currentPosition.getOrientation());
           }

           if(Math.abs(distanceToX) > maxorientationError + 20){
               fbpower = power * signX * Math.abs(Math.cos(Math.toRadians(currentPosition.getOrientation())));
           }else{
               fbpower = 0.3 * signX * Math.abs(Math.cos(Math.toRadians(currentPosition.getOrientation())));
           }

           if(Math.abs(distanceToY) > maxorientationError + 20){
               rlpower = power * signY * Math.abs(Math.sin(Math.toRadians(currentPosition.getOrientation())));
           }else{
               rlpower = 0.3 * signY * Math.abs(Math.sin(Math.toRadians(currentPosition.getOrientation())));
           }

           telemetry.addData("fbPower", fbpower);
           telemetry.addData("rlPower", rlpower);
           telemetry.addData("distance to x", distanceToX);
           telemetry.addData("distance to Y", distanceToY);

           move(fbpower, rlpower, compensatePower);
       }
       telemetry.addData("done", "yees");
       telemetry.update();
       stopChassis();
       goToDegrees(originalOrientation, 0.5);
       postCurrentPosition();
   }

    public void goToDistanceX(double x, double power){
        double currentX = currentPosition.getXPosition();
        int originalOrientation = currentPosition.getOrientation();

        double distanceToX = currentX - x;
        double compensateDegrees = 0;

        double signX = distanceToX/Math.abs(distanceToX);
        int orientationSign = 0;

        forward(0.5);

        while(Math.abs(distanceToX) > maxdistanceError){
            postCurrentPosition();
            currentX = currentPosition.getXPosition();

            distanceToX = currentX - x;

            signX = distanceToX/Math.abs(distanceToX);

            if(Math.abs(distanceToX) > maxorientationError + 20){
                move(power * signX,0,0);
            }else if((Math.abs(distanceToX) <= maxorientationError + 20) && (Math.abs(distanceToX) > maxorientationError + 10)){
                move(0.5 * signX,0,0);
            }else{
                move(0.2 * signX,0,0);
            }

            telemetry.addData("distance to x", distanceToX);
        }
        telemetry.addData("done", "yees");
        telemetry.update();
        postCurrentPosition();
        stopChassis();
        goToDegrees(originalOrientation, 0.5);
    }

   public void goToDegrees(int degrees, double power){
       int currentDegrees = currentPosition.getOrientation();
       int degreesToTheta = currentDegrees + degrees;

       int orientationSign = degrees == 0 ? 0 : degrees/Math.abs(degrees);

       while(Math.abs(degreesToTheta) > maxorientationError){

           currentDegrees = currentPosition.getOrientation();
           degreesToTheta = currentDegrees + degrees;
           orientationSign = degrees/Math.abs(degrees);

           if(Math.abs(degreesToTheta) > maxorientationError + 30){
               move(0,0,power * orientationSign);
           }else if((Math.abs(degreesToTheta) <= maxorientationError + 30) && (Math.abs(degreesToTheta) > maxorientationError + 10)){
               move(0,0,0.5 * orientationSign);
           }else{
               move(0,0,0);
           }
           postCurrentOrientation();
           telemetry.addData("degrees to theta", degreesToTheta);
           telemetry.addData("", "");
       }
       postCurrentOrientation();
       stopChassis();
   }

//   public void goToPosition(Position position, double power){
//       double currentX = currentPosition.getXPosition();
//       double currentY = currentPosition.getYPosition();
//       double currentOrientation = currentPosition.getOrientation();
//
//       double x = position.getXPosition();
//       double y = position.getYPosition();
//
//       double distanceToX = currentX - x;
//       double distanceToY = currentY - y;
//
//       double fbpower = 0;
//       double rlpower = 0;
//       double compensate = 0;
//
//       int signX = (int) (distanceToX/Math.abs(distanceToX));
//       int signY = (int) (distanceToY/Math.abs(distanceToY));
//       telemetry.addData("currentX", currentX);
//       telemetry.addData("positionX", x);
//
//       telemetry.addData("currentY", currentY);
//       telemetry.addData("positionY", y);
//
//       forward(0.5);
//
//       while((Math.abs(distanceToX) > maxdistanceError) || (Math.abs(distanceToY) > maxdistanceError)){
//           if(currentPosition.getOrientation() != currentOrientation){
//               compensate = (currentPosition.getOrientation() == 0
//                       ? currentPosition.getOrientation() / Math.abs(currentPosition.getOrientation())
//                       : 0) * 0.2;
//           }
//           currentX = currentPosition.getXPosition();
//
//           currentY = currentPosition.getYPosition();
//
//           distanceToX = currentX - x;
//           distanceToY = currentY - y;
//
//           signX = (int) (distanceToX/Math.abs(distanceToX));
//           signY = -(int) (distanceToY/Math.abs(distanceToY));
//
//           if(Math.abs(distanceToX) > maxdistanceError + 25){
//               fbpower = power * signX;
//           }else if((Math.abs(distanceToX) >= maxdistanceError + 25) && (Math.abs(distanceToX) < maxdistanceError + 10)){
//               fbpower = 0.3 * signX;
//           }else{
//               fbpower = 0.25 * signX;
//           }
//
//           if(Math.abs(distanceToY) > maxdistanceError + 25){
//               rlpower = power * signY;
//           }else if((Math.abs(distanceToY) >= maxdistanceError + 25) && (Math.abs(distanceToY) < maxdistanceError + 10)){
//               rlpower = 0.3 * signY;
//           }else{
//               rlpower = 0.25 * signY;
//           }
//
//           move(fbpower, rlpower, compensate);
//           telemetry.addData("distance to x", distanceToX);
//           telemetry.addData("", "");
//           postCurrentPosition();
//       }
//       stopChassis();
//       postCurrentPosition();
//   }

   public void turnDegrees(double desiredDegrees, double power){
       double currentOrientation = currentPosition.getOrientation();

       double degreesToOrientation = currentOrientation - desiredDegrees;
       int sign = (int) (degreesToOrientation/Math.abs(degreesToOrientation));
       double turnPower = 0;
       while(Math.abs(degreesToOrientation) > maxorientationError){
           //TODO: it loops here <-ERROR

           sign = (int) (degreesToOrientation/Math.abs(degreesToOrientation));
           if(Math.abs(degreesToOrientation) > maxorientationError + 50){
               move(0, 0, power * sign);
           }else{
               move(0, 0, 0.2 * sign);
           }

           telemetry.addData("degreesToOrientation", degreesToOrientation);
           postCurrentOrientation();
       }
       move(0, 0, 0);
       postCurrentOrientation();
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
