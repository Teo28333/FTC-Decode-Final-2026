package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private static final double CURRENT_FILTER_ALPHA = 0.2;
    private static final double SPIKE_HOLD_MS_MOTOR_1 = 500.0;
    private static final double SPIKE_HOLD_MS_MOTOR_2 = 250.0;
    private static final double RECOVERY_HOLD_MS = 400.0;
    private static final double RECOVERY_THRESHOLD_SCALE = 0.7;

    private final DcMotorEx intakeMotor1;
    private final DcMotorEx intakeMotor2;
    private final Servo gateServo;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;

    private final ElapsedTime gateFailSafe = new ElapsedTime();
    private final ElapsedTime startupTimer = new ElapsedTime();
    private final ElapsedTime spikeTimer1 = new ElapsedTime();
    private final ElapsedTime spikeTimer2 = new ElapsedTime();
    private final ElapsedTime recoverTimer1 = new ElapsedTime();
    private final ElapsedTime recoverTimer2 = new ElapsedTime();

    private double pow1 = 0.0, pow2 = 0.0;
    private double servoPos;

    private double current = 0.0;
    private double finalBallCurrent = 0.0;
    private double filteredCurrent = 0.0;
    private double filteredFinalBallCurrent = 0.0;

    private boolean spiked1 = false;
    private boolean spiked2 = false;
    private boolean inSpikeWindow1 = false;
    private boolean inSpikeWindow2 = false;
    private boolean wasIntakeActive = false;
    private boolean wasSpikeDetectionMode = false;

    private boolean activateIntake = false;
    private boolean activateAutonIntake = false;
    private boolean activateAutonTransfer = false;
    private boolean activateOuttake = false;
    private boolean activateTransfer = false;
    private boolean turnPlease = false;

    public Intake(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.dashboard = FtcDashboard.getInstance();

        intakeMotor1 = hw.get(DcMotorEx.class, "intakeMotor1");
        intakeMotor2 = hw.get(DcMotorEx.class, "intakeMotor2");

        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        gateServo = hw.get(Servo.class, "Blocker");
        servoPos = closeGatePos;
    }

    public void activateIntake(boolean isTrue) { activateIntake = isTrue; }
    public void activateAutonIntake(boolean isTrue) { activateAutonIntake = isTrue; }
    public void activateAutonTransfer(boolean isTrue) { activateAutonTransfer = isTrue; }
    public void activateOuttake(boolean isTrue) { activateOuttake = isTrue; }
    public void activateTransfer(boolean isTrue) { activateTransfer = isTrue; }

    public void read() {
        current = intakeMotor2.getCurrent(CurrentUnit.MILLIAMPS);
        finalBallCurrent = intakeMotor1.getCurrent(CurrentUnit.MILLIAMPS);

        filteredCurrent += CURRENT_FILTER_ALPHA * (current - filteredCurrent);
        filteredFinalBallCurrent += CURRENT_FILTER_ALPHA * (finalBallCurrent - filteredFinalBallCurrent);
    }

    public void update(double x, double y, boolean ready, boolean pointingCorrectly) {
        boolean isInZone = isInShootingZone(x, y);
        boolean spikeDetectionMode = activateIntake || activateAutonIntake;

        turnPlease = false;
        pow1 = 0;
        pow2 = 0;

        if (spikeDetectionMode && !wasSpikeDetectionMode) {
            spiked1 = false;
            spiked2 = false;
            inSpikeWindow1 = false;
            inSpikeWindow2 = false;
            startupTimer.reset();
            spikeTimer1.reset();
            spikeTimer2.reset();
            recoverTimer1.reset();
            recoverTimer2.reset();
            filteredCurrent = current;
            filteredFinalBallCurrent = finalBallCurrent;
        }
        wasSpikeDetectionMode = spikeDetectionMode;

        if (activateAutonTransfer || (activateTransfer && isInZone) || activateOuttake) {
            spiked1 = false;
            spiked2 = false;
            inSpikeWindow1 = false;
            inSpikeWindow2 = false;
            recoverTimer1.reset();
            recoverTimer2.reset();
            wasIntakeActive = false;
        }

        if (activateAutonTransfer) {
            openGate();
            double speed = isInBackZone(x, y) ? farTransferSpeed : closeTransferSpeed;
            pow1 = speed;
            pow2 = speed;
        }

        else if (activateAutonIntake) {
            closeGate();
            handleIntakeSpikes();
            pow1 = spiked1 ? 0 : intakeSpeed;
            pow2 = spiked2 ? 0 : intakeSpeed * 2.0 / 3.0;
        }

        else if (activateTransfer && isInZone) {
            openGate();

            if (!ready) {
                pow1 = 0.0;
                pow2 = 0.0;
            } else if (!pointingCorrectly) {
                turnPlease = true;
                pow1 = 0.0;
                pow2 = 0.0;
            } else {
                double speed = isInBackZone(x, y) ? farTransferSpeed : closeTransferSpeed;
                pow1 = speed;
                pow2 = speed;
            }
        }

        else if (activateIntake) {
            closeGate();
            handleIntakeSpikes();
            pow1 = spiked1 ? 0 : intakeSpeed;
            pow2 = spiked2 ? 0 : intakeSpeed * 2.0 / 3.0;
        }

        else if (activateOuttake) {
            openGate();
            pow1 = outakeSpeed;
            pow2 = outakeSpeed;
        }

        else {
            wasIntakeActive = false;
            wasSpikeDetectionMode = false;
            if (gateFailSafe.seconds() > 1) {
                closeGate();
            }
        }

        telemetry.addData("pow1", pow1);
        telemetry.addData("pow2", pow2);
        telemetry.addData("gate", servoPos);
        telemetry.addData("ready", ready);
        telemetry.addData("pointingCorrectly", pointingCorrectly);
        telemetry.addData("turnPlease", turnPlease);
        telemetry.addData("spiked1", spiked1);
        telemetry.addData("spiked2", spiked2);
        telemetry.addData("currentRaw", current);
        telemetry.addData("currentFiltered", filteredCurrent);
        telemetry.addData("finalCurrentRaw", finalBallCurrent);
        telemetry.addData("finalCurrentFiltered", filteredFinalBallCurrent);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("pow1", pow1);
        packet.put("pow2", pow2);
        packet.put("gate", servoPos);
        packet.put("current", current);
        packet.put("finalCurrent", finalBallCurrent);
        packet.put("ready", ready);
        packet.put("pointingCorrectly", pointingCorrectly);
        packet.put("turnPlease", turnPlease);
        packet.put("spiked1", spiked1);
        packet.put("spiked2", spiked2);
        packet.put("currentRaw", current);
        packet.put("currentFiltered", filteredCurrent);
        packet.put("finalCurrentRaw", finalBallCurrent);
        packet.put("finalCurrentFiltered", filteredFinalBallCurrent);
        dashboard.sendTelemetryPacket(packet);
    }

    private void handleIntakeSpikes() {
        if (!wasIntakeActive) {
            startupTimer.reset();
            inSpikeWindow1 = false;
            inSpikeWindow2 = false;
            recoverTimer1.reset();
            recoverTimer2.reset();
            wasIntakeActive = true;
        }

        if (startupTimer.milliseconds() <= 500) return;

        if (!spiked1) {
            if (filteredFinalBallCurrent > finalBallCurrentSpike) {
                if (!inSpikeWindow1) {
                    spikeTimer1.reset();
                    inSpikeWindow1 = true;
                } else if (spikeTimer1.milliseconds() > SPIKE_HOLD_MS_MOTOR_1) {
                    spiked1 = true;
                    recoverTimer1.reset();
                }
            } else {
                inSpikeWindow1 = false;
            }
        } else if (filteredFinalBallCurrent < finalBallCurrentSpike * RECOVERY_THRESHOLD_SCALE) {
            if (recoverTimer1.milliseconds() > RECOVERY_HOLD_MS) {
                spiked1 = false;
                inSpikeWindow1 = false;
            }
        } else {
            recoverTimer1.reset();
        }

        if (!spiked2) {
            if (filteredCurrent > currentSpike) {
                if (!inSpikeWindow2) {
                    spikeTimer2.reset();
                    inSpikeWindow2 = true;
                } else if (spikeTimer2.milliseconds() > SPIKE_HOLD_MS_MOTOR_2) {
                    spiked2 = true;
                    recoverTimer2.reset();
                }
            } else {
                inSpikeWindow2 = false;
            }
        } else if (filteredCurrent < currentSpike * RECOVERY_THRESHOLD_SCALE) {
            if (recoverTimer2.milliseconds() > RECOVERY_HOLD_MS) {
                spiked2 = false;
                inSpikeWindow2 = false;
            }
        } else {
            recoverTimer2.reset();
        }
    }

    private void openGate() {
        servoPos = openGatePos;
        gateFailSafe.reset();
    }

    private void closeGate() {
        servoPos = closeGatePos;
    }

    public static boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

    public static boolean isInBackZone(double x, double y) {
        double d = radius * Math.sqrt(2);
        return y <= x - 48 + d && y <= -x + 96 + d;
    }

    public static boolean isInFrontZone(double x, double y) {
        double d = radius * Math.sqrt(2);
        return y >= -x + 144 - d && y >= x - d;
    }

    public boolean turnPlease() {
        return turnPlease;
    }

    public void write() {
        intakeMotor1.setPower(pow1);
        intakeMotor2.setPower(pow2);
        gateServo.setPosition(servoPos);
    }
}
