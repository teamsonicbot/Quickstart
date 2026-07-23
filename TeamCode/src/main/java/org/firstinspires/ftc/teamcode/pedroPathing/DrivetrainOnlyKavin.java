package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous (name = "DrivetrainOnlyKavin", group = "Autonomous", preselectTeleOp = "TELEOP_JAVA")
public class DrivetrainOnlyKavin extends OpMode {


    private Follower follower;
    private boolean pathStarted = false;


    private Timer pathTimer;
    private Timer opModeTimer;
    private Paths paths;

        public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;


        public Paths(Follower follower) {


            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(56.168, 50.638)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();


            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.168, 50.638),
                                    new Pose(44.999, 35.000),
                                    new Pose(30.673, 74.181)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30.673, 74.181),
                                    new Pose(45.006, 94.972),
                                    new Pose(45.772, 64.279)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();


            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(45.772, 64.279),
                                    new Pose(52.243, 45.529),
                                    new Pose(33.518, 35.725)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(33.518, 35.725),
                                    new Pose(22.539, 19.411),
                                    new Pose(57.506, 27.421),
                                    new Pose(42.517, 11.097)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    public enum PathState {
        DriveTo_1,
        Launch_Element_1,
        DriveTo_2Pickup,
        DriveTo_3,
        Launch_Element_2,
        DriveTo_4Pickup,
        DriveTo_5,
        Launch_Element_3,
        IDLE
    }

    private PathState pathState = PathState.DriveTo_1;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    private void buildPaths () {
        paths = new Paths(follower);
    }


    private void statePathUpdate(){
        switch (pathState){


            case DriveTo_1:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path1, 0.7, true);   // Send Path1 to motors, cap speed at 70%, hold point when done
                    pathTimer.resetTimer();
                    pathStarted = true;
                }


                // Because this code runs every single frame while driving Path 1, can check pathTimer here
                // Example: If wanted to spin up the launcher halfway through driving:


                // Transition Check: If robot is finished moving AND path started AND safety delay is satisfied
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.DriveTo_2Pickup);
                }
                break;




            case DriveTo_2Pickup:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path2, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }




                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DriveTo_3);
                }
                break;


            case DriveTo_3:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path3, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }

                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DriveTo_4Pickup);
                }
                break;





            case DriveTo_4Pickup:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path4, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }




                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DriveTo_5);
                }
                break;



            case DriveTo_5:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path5, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }




                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.IDLE);
                }
                break;




            case IDLE:

                break;
        }
    }



    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;

    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);



        buildPaths();
        follower.setPose(startPose);
    }


    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = PathState.DriveTo_1;
        pathStarted = false;


    }


    @Override
    public void loop() {
        follower.update();

        statePathUpdate();


        telemetry.addData("Current Step", pathState);
        telemetry.addData("Robot Pose", follower.getPose().toString());
        telemetry.addData("Path Started", pathStarted);
        telemetry.addData("Follower Busy", follower.isBusy());

        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
