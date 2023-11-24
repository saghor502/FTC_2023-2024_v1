package org.firstinspires.ftc.teamcode.robot.tlop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "fieldCentricTLOP")
public class FieldCentric extends LinearOpMode {
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;

    BNO055IMU imu;
    Orientation angles = new Orientation();

    double initYaw;
    double adjustedJaw;
    public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        waitForStart();
        while(opModeIsActive()){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            adjustedJaw = angles.firstAngle - initYaw;
            double zerodYaw = -initYaw+ angles.firstAngle;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y,x) * 180/Math.PI;
            double realTheta;
            realTheta = (360 - zerodYaw) + theta;
            double power = Math.hypot(x,y);
            double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
            double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFrontOli = (power * cos / maxSinCos + turn);
            double rightFrontOli = (power * sin / maxSinCos - turn);
            double leftBackOli = (power * sin / maxSinCos + turn);
            double rightBackOli = (power * cos / maxSinCos - turn);
            if ((power + Math.abs(turn)) > 1) {
                leftFrontOli /= power + turn;
                rightFrontOli /= power - turn;
                leftBackOli /= power + turn;
                rightBackOli /= power - turn;
            }
            leftFront.setPower(leftFrontOli);
            rightFront.setPower(rightFrontOli);
            leftBack.setPower(leftBackOli);
            rightBack.setPower(rightBackOli);
        }
    }
}
