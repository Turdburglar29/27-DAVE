package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class GardenBlue1 extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(79.2689, 132.8973, 90);
    private final Pose path1 = p.of(79.2689, 129.4529, 90);
    private final Pose point2 = p.of(133.372, 133.304, -0.1927);
    private final Pose point2Control1 = p.of(89.62, 133.4716, 0);
    private final Pose point3Start = p.of(133.372, 133.304, 0);
    private final Pose point3 = p.of(79.2378, 129.7942, 90);
    private final Pose point3Control1 = p.of(118.2524, 125.5067, 0);
    private final Pose point4 = p.of(130.7333, 93.9462, -178.6933);
    private final Pose point4Control1 = p.of(105.3013, 93.1791, 0);
    private final Pose point5 = p.of(129.5724, 52.2778, -86.2852);
    private final Pose point5Control1 = p.of(94.2876, 96.6773, 0);
    private final Pose point5Control2 = p.of(129.1218, 64.2578, 0);


    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }
    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).tangent();
    }
    public Path path3() {return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);}
    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }
    public Path path5() {return Paths.curve(point4, point5Control1, point5Control2, point5).tangent();}

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                // Add shooting commands here
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
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
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
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
