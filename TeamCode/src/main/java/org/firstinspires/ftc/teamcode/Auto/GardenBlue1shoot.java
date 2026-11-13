package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class GardenBlue1shoot extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private DcMotor intake;
    private DcMotorEx shooter1;
    private Servo ballholder;

    private double targetVelocity = 300;
    private double idleVelocity = 300;

    private double lastError = 0;
    private long lastTime = System.nanoTime();

    private boolean firstShotDone = false;
    private boolean feeding = false;
    private boolean autoContinued = false;

    private boolean intakeStoppedAfterPath2 = false;

    private boolean secondShotDone = false;
    private boolean secondShotFeeding = false;

    private long secondShotStartTime = 0;
    private boolean secondAutoScheduled = false;

    private long shotStartTime = 0;

    private final Pose start = p.of(79.2689, 132.8973, 90);
    private final Pose path1 = p.of(79.2689, 129.4529, 90);
    private final Pose point2Start = p.of(79.2689, 129.4529, 90);
    private final Pose point2 = p.of(136, 132, 70);
    private final Pose point2Control1 = p.of(111.85, 125.55, 70);
    private final Pose point3Start = p.of(138.3267, 133.2436, 90);
    private final Pose point3 = p.of(79.2378, 129.7942, 90);
    private final Pose point3Control1 = p.of(118.2524, 125.5067, 0);
    private final Pose point4Start = p.of(79.2378, 129.7942, 90);
    private final Pose point4 = p.of(130.7333, 93.9462, 0);
    private final Pose point4Control1 = p.of(105.3013, 93.1791, 0);
    private final Pose point5 = p.of(130.3641, 45.7782, 93.8461);
    private final Pose point5Control1 = p.of(94.2876, 96.6773, 0);
    private final Pose point5Control2 = p.of(129.1218, 64.2578, 0);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5Control2, point5).reverseTangent();
    }

    private Command autoRoutine() {return sequential(follow(follower, path1()));
    }
    private double computeFlywheelPower(double targetVel, double measuredVel) {
            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            double error = targetVel - measuredVel;
            double derivative = (error - lastError) / Math.max(dt, 0.001);
            double kF = 0.34 / 800.0;
            double kP = 0.00025;
            double kD = 0.00001;
            double power = (kF * targetVel) + (kP * error) + (kD * derivative);
            lastError = error;lastTime = now;
            return Math.max(-1.0, Math.min(1.0, power));
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballholder = hardwareMap.get(Servo.class, "ballholder");
        ballholder.setPosition(0.6);
        follower.setPose(start);
        follower.update();
    }
    @Override
    public void start() {
        targetVelocity = 740;
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
// Shooter PID set power
        double shooterPower = computeFlywheelPower(targetVelocity, shooter1.getVelocity());
        shooter1.setPower(shooterPower);
// FIRST SHOT
        if (!firstShotDone && !follower.isBusy() && shooter1.getVelocity() >= 720 && shooter1.getVelocity() <= 740) {
            ballholder.setPosition(0.6);
            intake.setPower(1);
            shotStartTime = System.currentTimeMillis();
            feeding = true;
            firstShotDone = true;
        }
 // Finish first shot timer
        if (feeding && System.currentTimeMillis() - shotStartTime > 600) {
            feeding = false;
// Start intake for collection
            shooter1.setPower(idleVelocity);
            intake.setPower(1);
            ballholder.setPosition(0.1);
        }
// Start Path 2 and Path 3 only
        if (firstShotDone && !feeding && !autoContinued) {
            ballholder.setPosition(0.1);
            autoContinued = true;
            schedule(sequential(follow(follower, path2()), follow(follower, path3())));
        }
// STOP INTAKE AFTER PATH 2
        if (!intakeStoppedAfterPath2 && autoContinued && follower.pathIndex() == 1) {
            intake.setPower(1);
            ballholder.setPosition(0.1);
            intakeStoppedAfterPath2 = true;
        }
// SECOND SHOT AFTER PATH 3
        if (intakeStoppedAfterPath2 && !secondShotDone && !follower.isBusy() && shooter1.getVelocity() >= 720 && shooter1.getVelocity() <= 760) {
            ballholder.setPosition(0.6);
            intake.setPower(1);
            secondShotStartTime = System.currentTimeMillis();
            secondShotFeeding = true;
            secondShotDone = true;
        }
// Finish second shot timer
        if (secondShotFeeding && System.currentTimeMillis() - secondShotStartTime > 500) {
            shooter1.setPower(0);
            intake.setPower(0);
            ballholder.setPosition(0.1);
            secondShotFeeding = false;
            if (!secondAutoScheduled) {
                secondAutoScheduled = true;
                schedule(sequential(follow(follower, path4()), follow(follower, path5())));
            }
        }
        telemetry.addData("Velocity", shooter1.getVelocity());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Path Index", follower.pathIndex());
        telemetry.addData("First Shot Done", firstShotDone);
        telemetry.addData("Second Shot Done", secondShotDone);
        telemetry.addData("Intake Stopped", intakeStoppedAfterPath2);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
    @Override
    public void stop() {
        shooter1.setPower(0);
        OpModeStorage.autonomousEndPose = follower.pose();
    }
}