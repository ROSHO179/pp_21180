package org.firstinspires.ftc.teamcode.pedro.procedures.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    // Rates in ticks/sec per second; tune live via telemetry (target vs actual).
    public static double SPOOL_UP_RATE = 1500.0;
    public static double SPOOL_DOWN_RATE = 3000.0;
    public static double FULL_SPEED_TICKS_PER_SEC = 2000.0;

    public static double SWITCH_OPEN = 0.0;
    public static double SWITCH_CLOSE = 0.25;

    private final DcMotorEx launcherMotor;
    private final DcMotorEx launcherMotor2;
    private final Servo switchServo;

    double targetVelocity = 0;
    double commandedVelocity = 0;

    public Launcher(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class, "launcherR");
        switchServo = hardwareMap.get(Servo.class, "switchServo");

        for (DcMotorEx motor : new DcMotorEx[]{launcherMotor, launcherMotor2}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setTargetVelocity(double vel) {
        targetVelocity = vel;
    }

    public void spoolUp() {
        setTargetVelocity(FULL_SPEED_TICKS_PER_SEC);
    }

    public void spoolDown() {
        setTargetVelocity(0);
    }

    /** Slew commandedVelocity toward targetVelocity and write it. Call every loop. */
    public void update(double dt) {
        if (dt <= 0) dt = 1e-6;

        boolean crossingZero = commandedVelocity != 0 && Math.signum(targetVelocity) != Math.signum(commandedVelocity);
        if (crossingZero) {
            double step = SPOOL_DOWN_RATE * dt;
            commandedVelocity = Math.abs(commandedVelocity) <= step ? 0 : commandedVelocity - Math.signum(commandedVelocity) * step;
        } else {
            double error = targetVelocity - commandedVelocity;
            boolean increasing = Math.abs(targetVelocity) > Math.abs(commandedVelocity);
            double maxStep = (increasing ? SPOOL_UP_RATE : SPOOL_DOWN_RATE) * dt;
            commandedVelocity += Math.max(-maxStep, Math.min(maxStep, error));
        }

        launcherMotor.setVelocity(-commandedVelocity);
        launcherMotor2.setVelocity(commandedVelocity);
    }

    public boolean isAtSpeed(double tol) {
        return Math.abs(getMeasuredVelocity() - targetVelocity) <= tol;
    }

    /** Average flywheel velocity in the launcherMotor2 (positive) sign convention. */
    public double getMeasuredVelocity() {
        return (-launcherMotor.getVelocity() + launcherMotor2.getVelocity()) / 2.0;
    }

    public double getLeftVelocity() {
        return launcherMotor.getVelocity();
    }

    public double getRightVelocity() {
        return launcherMotor2.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCommandedVelocity() {
        return commandedVelocity;
    }

    /** Bypasses ramping; prefer setTargetVelocity/spoolUp/spoolDown + update(dt). */
    public void setLauncherVelocity(double velocity) {
        targetVelocity = velocity;
        commandedVelocity = velocity;
        launcherMotor.setVelocity(-velocity);
        launcherMotor2.setVelocity(velocity);
    }

    /** Bypasses ramping, and is overwritten by the next update(dt) call. */
    public void setLauncherPower(double power) {
        launcherMotor.setPower(-power);
        launcherMotor2.setPower(power);
    }

    public void openSwitch() {
        switchServo.setPosition(SWITCH_OPEN);
    }

    public void closeSwitch() {
        switchServo.setPosition(SWITCH_CLOSE);
    }
}
