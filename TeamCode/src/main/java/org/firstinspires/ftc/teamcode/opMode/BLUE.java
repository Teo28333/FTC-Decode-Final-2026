package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.blueGoalX;
import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.goalY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp
public class BLUE extends OpMode {
    private static final double STICK_DEADBAND = 0.05;
    private static final double SLOW_MODE_SCALE = 0.45;
    private static final double AIM_ASSIST_SCALE = 0.8;

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Lift lift;
    private Limelight3A ll3a;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap);
        //ll3a = hardwareMap.get(Limelight3A.class, "limelight");
        //ll3a.pipelineSwitch(1);
        //ll3a.setPollRateHz(20);
        //ll3a.start();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        follower.update();

        follower.setPose(new Pose(72, 72, 0));
    }

    @Override
    public void loop() {
        applyDriveCommand();
        applySubsystemCommands();
        runSubsystemLoop();
        telemetry.update();
        if (intake.turnPlease()) {
            turn = gamepad1.right_stick_x + shooter.getChassisTurnAssist();
        } else {
            turn = gamepad1.right_stick_x;
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -turn,
                false,
                Math.toRadians(180));

        shooter.offsetLess(gamepad1.dpad_left);
        shooter.offsetMore(gamepad1.dpad_right);
        shooter.resetOffset(gamepad1.dpad_down);

        intake.activateIntake(gamepad1.right_bumper);
        intake.activateTransfer(gamepad1.left_bumper);
        intake.activateOuttake(gamepad1.square);

        lift.activateLift(gamepad2.right_bumper);
        lift.activatePto(gamepad2.left_bumper);

        SSLoop();
    }

    private void runSubsystemLoop() {
        follower.update();

        shooter.read();
        shooter.update(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), blueGoalX, goalY, follower.getVelocity(), follower.getAngularVelocity());
        shooter.write();

        intake.read();
        intake.update(follower.getPose().getX(), follower.getPose().getY(), shooter.isReady(), shooter.isShooterPointingCorrectly());
        intake.write();

        lift.read();
        lift.update();
        lift.write();
    }

    private void applyDriveCommand() {
        double speedScale = gamepad1.right_trigger > 0.2 ? SLOW_MODE_SCALE : 1.0;

        double driveY = applyDeadband(-gamepad1.left_stick_y) * speedScale;
        double driveX = applyDeadband(-gamepad1.left_stick_x) * speedScale;
        double manualTurn = applyDeadband(gamepad1.right_stick_x) * speedScale;

        double turnAssist = 0.0;
        if (intake.turnPlease() && !gamepad1.cross) {
            turnAssist = shooter.getChassisTurnAssist() * AIM_ASSIST_SCALE;
        }

        follower.setTeleOpDrive(
                driveY,
                driveX,
                -(manualTurn + turnAssist),
                false,
                Math.toRadians(180)
        );
    }

    private void applySubsystemCommands() {
        shooter.offsetLess(gamepad1.dpad_left);
        shooter.offsetMore(gamepad1.dpad_right);
        shooter.resetOffset(gamepad1.dpad_down);

        intake.activateIntake(gamepad1.right_bumper);
        intake.activateTransfer(gamepad1.left_bumper);
        intake.activateOuttake(gamepad1.square);

        lift.activateLift(gamepad1.triangle);
        lift.activatePto(gamepad1.circle);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0.0;
    }

    private void relocalize() {
        LLResult result = ll3a.getLatestResult();
        ll3a.updateRobotOrientation(Math.toDegrees(follower.getHeading()) + 90);
        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMt2 = result.getBotpose_MT2();
                Pose3D botPoseMt1 = result.getBotpose();

                double x = botPoseMt2.getPosition().x;
                double y = botPoseMt2.getPosition().y;
                double heading = botPoseMt1.getOrientation().getYaw();

                follower.setPose(ll3aToPedro(x, y));
            }
        }
    }


    private Pose ll3aToPedro(double x, double y) {
        return new Pose(y * 39.3700787402 + 72, x * 39.3700787402 + 72);
    }
}
