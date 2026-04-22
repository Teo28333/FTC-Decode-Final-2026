package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private static final double PTO_ENGAGED_POS = 0.5;
    private static final double PTO_DISENGAGED_POS = 0.0;
    private static final double LIFT_POWER = 0.875;

    private final DcMotorEx bR;
    private final DcMotorEx bL;
    private final DcMotorEx fR;
    private final DcMotorEx fL;
    private final Servo ptoServo;

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

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        ptoServo.setPosition(ptoActivate ? PTO_ENGAGED_POS : PTO_DISENGAGED_POS);

        if (liftActivate) {
            backPow = LIFT_POWER;
            frontPow = -LIFT_POWER;
        } else {
            backPow = 0.0;
            frontPow = 0.0;
        }
    }

    public void write() {
        fL.setPower(frontPow);
        fR.setPower(frontPow);
        bL.setPower(backPow);
        bR.setPower(backPow);
    }
}
