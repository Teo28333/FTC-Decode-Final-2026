package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;
import org.firstinspires.ftc.teamcode.math.SpeedController;

public class Shooter {

    private final DcMotorEx motor1, motor2;
    private final Servo hoodServo;
    private final ServoImplEx turretServo1, turretServo2;

    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private final SpeedController speedController;
    private final ShooterEquation shooterEquation;

    private static final double TURRET_ALPHA = 0.12;
    private static final double TURRET_SWITCH_THRESHOLD = 10.0;
    private static final double TURRET_MOUNT_OFFSET_DEG = 180.0;

    private static final double TURRET_MIN = -160.0;
    private static final double TURRET_MAX = 160.0;

    private static final double SAFE_TURRET_MIN = -155.0;
    private static final double SAFE_TURRET_MAX = 155.0;

    private static final double CHASSIS_TURN_KP = 0.03;
    private static final double MAX_CHASSIS_TURN_ASSIST = 0.35;

    private static final double AIM_TOLERANCE_DEG = 2.0;

    private double rpm = 0.0;
    private double targetRPM = 0.0;
    private double hoodPos = 0.0;
    private double motorPow = 0.0;

    private double finalTurretAngle = 0.0;
    private double lastCommandedAngle = 0.0;
    private double turretOffset = 0.0;

    private double desiredTurretAngleDeg = 0.0;
    private double chassisTurnAssist = 0.0;
    private double turretAimErrorDeg = 0.0;

    private boolean offsetMore = false;
    private boolean offsetLess = false;
    private boolean resetOffset = false;
    private boolean isInTuningMode = false;

    public Shooter(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.dashboard = FtcDashboard.getInstance();

        motor1 = hw.get(DcMotorEx.class, "ShooterA");
        motor2 = hw.get(DcMotorEx.class, "ShooterB");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hw.get(Servo.class, "Hood");

        turretServo1 = hw.get(ServoImplEx.class, "tS1");
        turretServo2 = hw.get(ServoImplEx.class, "tS2");

        PwmControl.PwmRange fullRange = new PwmControl.PwmRange(500, 2500);
        turretServo1.setPwmRange(fullRange);
        turretServo2.setPwmRange(fullRange);

        speedController = new SpeedController(hw);
        speedController.reset();

        shooterEquation = new ShooterEquation();
    }

    public void offsetMore(boolean b) { offsetMore = b; }
    public void offsetLess(boolean b) { offsetLess = b; }
    public void resetOffset(boolean b) { resetOffset = b; }
    public void isInTuningMode(boolean b) { isInTuningMode = b; }

    public void read() {
        rpm = (motor1.getVelocity() / 28.0) * 60.0;

        telemetry.addData("RPM", rpm);
        telemetry.addData("Target RPM", targetRPM);
    }

    private double normalizeDeg(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double shortestAngleDiffDeg(double target, double current) {
        return normalizeDeg(target - current);
    }

    private double unwrapAngle(double target, double current) {
        double delta = target - current;

        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;

        return current + delta;
    }

    public void update(double x, double y, double heading,
                       double goalX, double goalY,
                       Vector velocity,
                       double omega) {

        double distanceNow = Math.hypot(goalX - x, goalY - y);
        double t = shooterEquation.getAirTime(distanceNow);

        double vx = velocity.getXComponent();
        double vy = velocity.getYComponent();

        double futureX = x + vx * t;
        double futureY = y + vy * t;

        Vector2d futureOffset = new Vector2d(xOffset, yOffset)
                .rotateBy(heading + omega * t);

        double shooterFutureX = futureX + futureOffset.getX();
        double shooterFutureY = futureY + futureOffset.getY();

        double dx = goalX - shooterFutureX;
        double dy = goalY - shooterFutureY;

        double distance = Math.hypot(dx, dy);
        double fieldAngle = Math.atan2(dy, dx);

        double rawAngle = -Math.toDegrees(fieldAngle)
                + Math.toDegrees(heading)
                + TURRET_MOUNT_OFFSET_DEG;

        rawAngle = normalizeDeg(rawAngle);

        if (resetOffset) {
            turretOffset = 0.0;
        } else if (offsetLess) {
            turretOffset -= 1.0;
        } else if (offsetMore) {
            turretOffset += 1.0;
        }

        desiredTurretAngleDeg = normalizeDeg(rawAngle + turretOffset);

        double overflowDeg = 0.0;

        if (desiredTurretAngleDeg > SAFE_TURRET_MAX) {
            overflowDeg = desiredTurretAngleDeg - SAFE_TURRET_MAX;
        } else if (desiredTurretAngleDeg < SAFE_TURRET_MIN) {
            overflowDeg = desiredTurretAngleDeg - SAFE_TURRET_MIN;
        }

        chassisTurnAssist = Range.clip(
                -overflowDeg * CHASSIS_TURN_KP,
                -MAX_CHASSIS_TURN_ASSIST,
                MAX_CHASSIS_TURN_ASSIST
        );

        double unwrapped = unwrapAngle(desiredTurretAngleDeg, finalTurretAngle);
        double delta = unwrapped - finalTurretAngle;

        if (Math.abs(delta) > 180.0 - TURRET_SWITCH_THRESHOLD) {
            if (delta > 0.0) unwrapped -= 360.0;
            else unwrapped += 360.0;
        }

        double commandedAngle = Range.clip(
                unwrapped,
                SAFE_TURRET_MIN,
                SAFE_TURRET_MAX
        );

        commandedAngle = Range.clip(commandedAngle, TURRET_MIN, TURRET_MAX);

        double smoothed = lastCommandedAngle
                + TURRET_ALPHA * (commandedAngle - lastCommandedAngle);

        finalTurretAngle = smoothed;
        lastCommandedAngle = smoothed;

        turretAimErrorDeg = shortestAngleDiffDeg(desiredTurretAngleDeg, finalTurretAngle);

        if (isInTuningMode) {
            targetRPM = tuningSpeed;
            hoodPos = tuningHoodPos;
        } else {
            targetRPM = shooterEquation.getTargetRPM(distance);
            hoodPos = shooterEquation.getHoodPos(distance);
        }

        motorPow = speedController.calculate(targetRPM, rpm);

        telemetry.addData("Turret Angle", finalTurretAngle);
        telemetry.addData("Desired Turret Angle", desiredTurretAngleDeg);
        telemetry.addData("Turret Aim Error", turretAimErrorDeg);
        telemetry.addData("Chassis Turn Assist", chassisTurnAssist);
        telemetry.addData("Distance", distance);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TurretAngle", finalTurretAngle);
        packet.put("DesiredTurretAngle", desiredTurretAngleDeg);
        packet.put("TurretAimError", turretAimErrorDeg);
        packet.put("ChassisTurnAssist", chassisTurnAssist);
        packet.put("Distance", distance);
        packet.put("rpm", rpm);
        packet.put("target rpm", targetRPM);

        dashboard.sendTelemetryPacket(packet);
    }

    public void write() {
        hoodServo.setPosition(Range.clip(hoodPos, 0.0, 1.0));

        motor1.setPower(Range.clip(motorPow, -1.0, 1.0));
        motor2.setPower(Range.clip(motorPow, -1.0, 1.0));

        double servoPos = angleDegToServoPos(finalTurretAngle);

        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    }

    public boolean isReady() {
        return Math.abs(rpm - targetRPM) <= rpmTol;
    }

    public double getChassisTurnAssist() {
        return chassisTurnAssist;
    }

    public double getTurretAimErrorDeg() {
        return turretAimErrorDeg;
    }

    public double getDesiredTurretAngleDeg() {
        return desiredTurretAngleDeg;
    }

    public double getTurretAngleDeg() {
        return finalTurretAngle;
    }

    public boolean isShooterPointingCorrectly() {
        return Math.abs(chassisTurnAssist) < 0.01
                && Math.abs(turretAimErrorDeg) <= AIM_TOLERANCE_DEG;
    }

    private double angleDegToServoPos(double angleDeg) {
        double pos = (angleDeg + maxAngle) / (maxAngle - minAngle);
        return Range.clip(pos, 0.0, 1.0);
    }
}