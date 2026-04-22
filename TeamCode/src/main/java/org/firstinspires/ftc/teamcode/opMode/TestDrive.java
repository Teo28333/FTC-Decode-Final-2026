package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pedro Test Drive", group = "Test")
public class TestDrive extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("Pedro Test Drive initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(new Pose(72, 72, 0));
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                y,
                x,
                turn,
                false,
                0
        );

        follower.update();

        telemetry.addData("x input", x);
        telemetry.addData("y input", y);
        telemetry.addData("turn input", turn);
        telemetry.addData("pose x", follower.getPose().getX());
        telemetry.addData("pose y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }
}