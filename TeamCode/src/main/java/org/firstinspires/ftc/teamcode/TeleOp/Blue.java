
package org.firstinspires.ftc.teamcode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(name = "BlueCLose")
public class Blue extends OpMode {
    public static DcMotor intake;
    private Follower follower;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
    private Servo ballholder;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        ballstopper = hardwareMap.get(DcMotor.class, "ballstopper");
        ballholder = hardwareMap.get(Servo.class, "ballholder");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballstopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        ballstopper.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
    }
    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }
    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        // shoot button
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else {intake.setPower(0);}

        if (gamepad1.left_bumper) {
            intake.setPower(-1);
            ballholder.setPosition(0);
        } else {ballholder.setPosition(.4);}

        if (gamepad1.cross) {
            ((DcMotorEx) shooter1).setVelocity(800);
            ((DcMotorEx) shooter2).setVelocity(800);
            if ((((DcMotorEx) shooter1).getVelocity() >= 780) && (((DcMotorEx) shooter1).getVelocity() >= 820)) {
                intake.setPower(-1);
            } else {intake.setPower(0);}}

        else if (gamepad1.circle) {
            ((DcMotorEx) shooter1).setVelocity(775);
            ((DcMotorEx) shooter2).setVelocity(775);
            if ((((DcMotorEx) shooter1).getVelocity() >= 755) && (((DcMotorEx) shooter1).getVelocity() >= 795)){
                intake.setPower(-1);
            } else {intake.setPower(0);}}

        else if (gamepad1.triangle) {
            ((DcMotorEx) shooter1).setVelocity(750);
            ((DcMotorEx) shooter2).setVelocity(750);
            if ((((DcMotorEx) shooter1).getVelocity() >= 730) && (((DcMotorEx) shooter1).getVelocity() >= 770)) {
                intake.setPower(-1);
            }else {intake.setPower(0);}}

        else {
            ((DcMotorEx) shooter1).setVelocity(0);
            ((DcMotorEx) shooter2).setVelocity(0);
            intake.setPower(0);
        }
                // relocalise button
        if (gamepad1.optionsWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(270));
            // On the fly Pose creation, we don't recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }

        follower.update();
        Pose robotPose = follower.pose();

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("shooter", ((DcMotorEx) shooter1).getVelocity());
    }
}