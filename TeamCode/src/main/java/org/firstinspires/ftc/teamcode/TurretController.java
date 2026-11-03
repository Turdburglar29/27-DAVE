package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretController {

    public static final double GOAL_X = 62.5;
    public static final double GOAL_Y = 62.5;

    public static final double TURRET_TICKS_PER_REV = 726;
    public static final double TURRET_TICKS_PER_RAD =
            TURRET_TICKS_PER_REV / (2.0 * Math.PI);

    private static double mountOffsetRad = 0.0;
    private static int headingSign = -1;

    private static boolean swapXY = false;
    private static boolean invertX = true;
    private static boolean invertY = false;

    public static boolean USE_SOFT_LIMITS = false;
    public static double MIN_ANGLE_RAD = Math.toRadians(0);
    public static double MAX_ANGLE_RAD = Math.toRadians(50);

    private final DcMotorEx turretMotor;
    private final Follower follower;

    private double kP = 0.2;
    private double kD = 0.009;
    private double kS = 0.13;

    private double maxPower = 1.0;
    private double deadbandRad = Math.toRadians(1.5);

    private double lastError = 0.0;
    private long lastTimeNanos = 0L;

    private boolean holdAtZero = false;
    private int holdTargetTick = 0;

    private double hold_kP = 0.9;
    private double hold_kD = 0.01;
    private double hold_kS = 0.12;

    private boolean tickSoftLimitsEnabled = false;
    private int minTick = -1650;
    private int maxTick = 1025;
    private int softMarginTicks = 3;
    private int slowZoneTicks = 15;
    private double slowZoneMinScale = 0.3;

    public TurretController(HardwareMap hw, String turretMotorName, Follower follower) {
        this.follower = follower;
        turretMotor = hw.get(DcMotorEx.class, turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastTimeNanos = System.nanoTime();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double wrapRad(double angle) {
        while (angle >= Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
    private static double mapX(double x, double y) {
        double rx = swapXY ? y : x;
        if (invertX) {
            rx = -rx;
        }
        return rx;
    }

    private static double mapY(double x, double y) {
        double ry = swapXY ? x : y;
        if (invertY) {
            ry = -ry;
        }
        return ry;
    }

    private static double mappedFieldAngleToGoal(double robotX, double robotY) {
        double rx = mapX(robotX, robotY);
        double ry = mapY(robotX, robotY);

        double rgx = mapX(GOAL_X, GOAL_Y);
        double rgy = mapY(GOAL_X, GOAL_Y);

        return Math.atan2(rgy - ry, rgx - rx);
    }
    public void setInverted(boolean inverted) {
        turretMotor.setDirection(inverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }
    public void setHeadingCcwPositive(boolean ccwPositive) {
        headingSign = ccwPositive ? 1 : -1;
    }
    public void setMountOffsetRad(double offsetRad) {
        mountOffsetRad = offsetRad;
    }
    public void setSwapXY(boolean value) {
        swapXY = value;
    }
    public void setInvertX(boolean value) {
        invertX = value;
    }
    public void setInvertY(boolean value) {
        invertY = value;
    }
    public void setGains(double kP, double kD, double kS) {
        this.kP = kP;
        this.kD = kD;
        this.kS = kS;
    }
    public void setHoldAtZeroEnabled(boolean enabled) {
        holdAtZero = enabled;
        if (enabled) {
            lastTimeNanos =
                    System.nanoTime();
        }
    }
    public void setHoldTargetTick(
            int tick
    ) {
        holdTargetTick = tick;
    }
    public int getCurrentTick() {
        return turretMotor.getCurrentPosition();
    }
    public void setTickSoftLimitsEnabled(boolean enabled) {
        tickSoftLimitsEnabled = enabled;
    }

    public void setTickLimits(
            int minTick,
            int maxTick
    ) {
        this.minTick = minTick;
        this.maxTick = maxTick;
    }

    public void update() {

        Pose pose = follower.pose();

        double robotX = pose.x();
        double robotY = pose.y();
        double robotHeading = pose.heading();
        double fieldAngleToGoal =
                mappedFieldAngleToGoal(robotX, robotY);

        double desiredTurretAngleRad =
                wrapRad(fieldAngleToGoal - (headingSign * robotHeading) - mountOffsetRad);

        if (holdAtZero) {desiredTurretAngleRad = holdTargetTick / TURRET_TICKS_PER_RAD;
        }

        if (USE_SOFT_LIMITS) {desiredTurretAngleRad = clamp(desiredTurretAngleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        }
        final int currentTicks = turretMotor.getCurrentPosition();

        double currentAngleRad = currentTicks / TURRET_TICKS_PER_RAD;

        double error;

        if (tickSoftLimitsEnabled) {
            int desiredTicks = (int) Math.round(desiredTurretAngleRad * TURRET_TICKS_PER_RAD);
            desiredTicks = (int) clamp(desiredTicks, minTick, maxTick);
            desiredTurretAngleRad = desiredTicks / TURRET_TICKS_PER_RAD;
            error = desiredTurretAngleRad - currentAngleRad;}
        else {

            error = wrapRad(desiredTurretAngleRad - currentAngleRad);
        }

        if (Math.abs(error) <= deadbandRad) {

            turretMotor.setPower(0);

            lastError = error;
            lastTimeNanos =
                    System.nanoTime();

            return;
        }

        long now =
                System.nanoTime();

        double dt =
                Math.max(
                        0.0001,
                        (now - lastTimeNanos)
                                / 1e9
                );

        double derivative =
                (error - lastError)
                        / dt;

        double use_kP =
                holdAtZero
                        ? hold_kP
                        : kP;

        double use_kD =
                holdAtZero
                        ? hold_kD
                        : kD;

        double use_kS =
                holdAtZero
                        ? hold_kS
                        : kS;

        double power =
                use_kP * error
                        + use_kD * derivative
                        + use_kS * Math.signum(error);

        if (tickSoftLimitsEnabled) {

            if ((currentTicks - minTick)
                    <= softMarginTicks
                    && power < 0) {
                power = 0;
            }

            if ((maxTick - currentTicks)
                    <= softMarginTicks
                    && power > 0) {
                power = 0;
            }
        }

        power =
                clamp(
                        power,
                        -maxPower,
                        maxPower
                );

        turretMotor.setPower(power);

        lastError = error;
        lastTimeNanos = now;
    }

    public void calibrateMountOffsetToCurrentAim() {

        Pose pose = follower.pose();

        double robotX = pose.x();
        double robotY = pose.y();
        double robotHeading = pose.heading();

        double fieldAngleToGoal =
                mappedFieldAngleToGoal(
                        robotX,
                        robotY
                );

        double currentAngleRad =
                turretMotor.getCurrentPosition()
                        / TURRET_TICKS_PER_RAD;

        mountOffsetRad =
                wrapRad(
                        fieldAngleToGoal
                                - (headingSign * robotHeading)
                                - currentAngleRad
                );
    }
}