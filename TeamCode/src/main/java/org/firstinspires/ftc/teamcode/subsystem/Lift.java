package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private DcMotorEx bR, bL, fR, fL;
    private Servo ptoServo;

    private double backPow;
    private double frontPow;

    private boolean liftActivate;
    private boolean ptoActivate;

    public Lift(HardwareMap hw) {
        fL = hw.get(DcMotorEx.class, "frontL");
        bL = hw.get(DcMotorEx.class, "backL");
        fR = hw.get(DcMotorEx.class, "frontR");
        bR = hw.get(DcMotorEx.class, "backR");
        ptoServo = hw.get(Servo.class, "ptoS");

        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void activateLift(boolean isTrue) {
        liftActivate = isTrue;
    }

    public void activatePto(boolean isTrue) {
        ptoActivate = isTrue;
    }

    public void read() {

    }

    public void update() {

        if (ptoActivate) {
            ptoServo.setPosition(0.5);
        } else ptoServo.setPosition(0);

        if (liftActivate) {
            backPow = 0.875;
            frontPow = -875;
        }
    }

    public void write() {
        fL.setPower(frontPow);
        fR.setPower(frontPow);
        bL.setPower(backPow);
        bR.setPower(backPow);
    }
}
