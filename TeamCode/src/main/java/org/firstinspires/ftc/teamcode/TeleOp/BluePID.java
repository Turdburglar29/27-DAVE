package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.TurretController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BluePID")
public class BluePID extends OpMode {

    public static DcMotor intake;

    private Follower follower;

    private DcMotorEx shooter1;
    // private DcMotorEx shooter2;

    private Servo ballholder;
    private TurretController turret;

    // PIDF constants
    private double kF = 0.44 / 800.0; // (first number perecent power) / (velocity set)
    private double kP = 0.0000;
    private double kD = 0.00000;

    private double lastError = 0;
    private long lastTime = System.nanoTime();

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        ballholder = hardwareMap.get(Servo.class, "ballholder");

        turret = new TurretController(
                hardwareMap,
                "turret",
                follower
        );

        telemetry.update();
    }

    @Override
    public void start() {

        follower.setPose(
                OpModeStorage.autonomousEndPose
        );

        follower.update();
    }

    private double computeFlywheelPower(
            double targetVel,
            double measuredVel
    ) {

        long now = System.nanoTime();

        double dt =
                (now - lastTime) / 1e9;

        double error =
                targetVel - measuredVel;

        double derivative =
                (error - lastError)
                        / Math.max(dt, 0.001);

        double power =
                (kF * targetVel)
                        +
                        (kP * error)
                        +
                        (kD * derivative);

        lastError = error;
        lastTime = now;

        return Math.max(
                -1.0,
                Math.min(1.0, power)
        );
    }

    @Override
    public void loop() {

        DrivePowers powers =
                ManualDrive.fieldCentric(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        follower.pose().heading()
                );

        ManualDrive.driveOrHold(
                follower,
                powers
        );

        double targetVelocity = 0;

        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.left_bumper) {
            intake.setPower(-1);
            ballholder.setPosition(0);
        } else {
            ballholder.setPosition(.4);
        }

        if (gamepad1.cross) {
            targetVelocity = 800;
        }
        else if (gamepad1.circle) {
            targetVelocity = 775;
        }
        else if (gamepad1.triangle) {
            targetVelocity = 750;
        }

        if (targetVelocity > 0) {

            double measuredVelocity =
                    shooter1.getVelocity();

            double shooterPower =
                    computeFlywheelPower(
                            targetVelocity,
                            measuredVelocity
                    );

            shooter1.setPower(
                    shooterPower
            );

            if (Math.abs(
                    measuredVelocity
                            - targetVelocity
            ) <= 20) {

                intake.setPower(-1);

            } else {

                intake.setPower(0);
            }

        } else {

            shooter1.setPower(0);

            if (!gamepad1.right_bumper
                    && !gamepad1.left_bumper) {

                intake.setPower(0);
            }
        }

        if (gamepad1.optionsWasPressed()) {

            Pose cornerPose =
                    new Pose(
                            10.5,
                            10.5,
                            Math.toRadians(270)
                    );

            follower.setPose(
                    cornerPose
            );
        }

        follower.update();
        turret.update();

        telemetry.addData(
                "X",
                follower.pose().x()
        );

        telemetry.addData(
                "Y",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );

        telemetry.addData(
                "Velocity",
                shooter1.getVelocity()
        );

        telemetry.addData(
                "Target Velocity",
                targetVelocity
        );

        telemetry.addData(
                "Shooter Power",
                shooter1.getPower()
        );

        telemetry.update();
    }
}