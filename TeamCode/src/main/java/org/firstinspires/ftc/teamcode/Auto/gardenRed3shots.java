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
public class gardenRed3shots extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(57.405, 11.4121, 270);
    private final Pose path1 = p.of(10.8404, 11.1121, 270);
    private final Pose path1Control1 = p.of(30.5206, 51.0213, 0);
    private final Pose point2Start = p.of(10.8404, 11.1121, 270);
    private final Pose point2 = p.of(57.4369, 10.9277, 270);
    private final Pose point3Start = p.of(57.4369, 10.9277, 270);
    private final Pose point3 = p.of(10.7695, 47.334, 180);
    private final Pose point4Start = p.of(10.7695, 47.334, 180);
    private final Pose point4 = p.of(64.7638, 132.3957, 90);
    private final Pose point5Start = p.of(64.7638, 132.3957, 90);
    private final Pose point5 = p.of(59.3943, 132.7206, 90);
    private final Pose point6Start = p.of(59.3943, 132.7206, 90);
    private final Pose point6 = p.of(47.8099, 132.4901, 90);
    private final Pose point6Control1 = p.of(53.8688, 89.7085, 0);
    private final Pose point7Start = p.of(47.8099, 132.4901, 90);
    private final Pose point7 = p.of(12.8014, 120.378, 0);

    public Path path1() {return Paths.curve(start, path1Control1, path1).linear(start, path1);}

    public Path path2() {return Paths.line(point2Start, point2).linear(point2Start, point2);}

    public Path path3() {return Paths.line(point3Start, point3).linear(point3Start, point3);}

    public Path path4() {return Paths.line(point4Start, point4).linear(point4Start, point4);}

    public Path path5() {return Paths.line(point5Start, point5).linear(point5Start, point5);}
    public Path path6() {return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);}
    public Path path7() {return Paths.line(point7Start, point7).linear(point7Start, point7);}

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                // Add shooting commands here
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
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
