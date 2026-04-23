package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestLift extends OpMode {

    private DcMotorEx left, right;
    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "backL");
        right = hardwareMap.get(DcMotorEx.class, "backR");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad1.y) {
            left.setPower(1);
            right.setPower(1);
        } else if (gamepad1.x) {
            left.setPower(-0.75);
            right.setPower(-0.75);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
    }
}
