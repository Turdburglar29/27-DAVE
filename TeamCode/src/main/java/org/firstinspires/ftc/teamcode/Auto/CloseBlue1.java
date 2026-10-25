package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static java.util.concurrent.locks.LockSupport.park;

import android.os.DropBoxManager;

@Autonomous
public class CloseBlue1 extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startingSpot = p.of(80, 130, 90);
    private final Pose score = p.of(48, 48, 90);

    private final Pose control = p.of(135, 128, 90);
    private final Pose end = p.of(130, 48, 90);
    private TelemetryMessage telemetryData;

    public Path scorePreload() {
        return Paths.line(startingSpot, score);
    }
    public Path park() {
        return Paths.curve(score, control, end);
    }
    public Path compoundPath() {
        return Paths.path(scorePreload(), park());
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, scorePreload()),
                // Add shooting commands here
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingSpot);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        // We will add these two, but you can use the many other follower methods aswell in your own code!
        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining in path", follower.remainingDistance());
        telemetry.update();
    }
    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}
/*
public Path linePath() {
    return Paths.line(start, end);
}
public Path curvePath() {
    return Paths.curve(start, control, end);
}
public Path compoundPath() {
    return Paths.path(linePath(), curvePath());
}

follower.atParametricEnd() ---- When wanting to do something when it relatively meets end location or
Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4
!follower.isBusy() When wanting to do something when it meets official end location
follower.completion() gives you, from 0 to 1, the amount of the current path segment the robot has completed.
follower.remainingDistance() gives you the distance remaining along the path for the robot to travel.
follower.distanceToEndpoint() gives you the Cartesian distance from the robot to the endpoint of the path.
follower.closestPose() gives you the closest pose on the path to the robot.
follower.pathIndex() gives you the index of the current segment of the path you are following. For instance, if you've chained 3 paths together, it may tell you that you're following the second one.
follower.currentCurve() gives you the Curve object your robot is currently following.
follower.currentPath() gives you the Path object your robot is currently following.  */
