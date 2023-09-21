package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdoMath {
    public static double getDistance(double EncoderTicks){
        return(EncoderTicks * 0.01533980788);
    }

    public static class PosFuncv1 {
        public static void goToPosTest(double metrosX, double metrosY, DcMotor rf, DcMotor rb, DcMotor lf, DcMotor lb){

            double EncoderPosX;
            double EncoderPosY;
            EncoderPosX = metrosX * 6519.012;
            EncoderPosY = metrosY * 6519.012;
            double targetX = EncoderPosX - 100;
            double targetY = EncoderPosY - 100;
            if (EncoderPosX > 1) { //derecha
                rf.setPower(0.1);
                rb.setPower(-0.1);
                lf.setPower(-0.1);
                lb.setPower(0.1);
                if (EncoderPosX == targetX){
                    rf.setPower(0);
                    rb.setPower(0);
                    lf.setPower(0);
                    lb.setPower(0);
                }
            } else if (EncoderPosX < -1){
                rf.setPower(-0.1);
                rb.setPower(0.1);
                lf.setPower(0.1);
                lb.setPower(-0.1);
                if (EncoderPosX == targetX){
                    rf.setPower(0);
                    rb.setPower(0);
                    lf.setPower(0);
                    lb.setPower(0);
                }
            }
        }
    }
}
