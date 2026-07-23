package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherStates;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeStates;

@Autonomous(name = "Red2Auto", group = "Pedro", preselectTeleOp = "TELEOP_JAVA")
public class Red2Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LauncherStates launcher;
    private boolean flywheelStarted = false;
    private boolean pathStarted = false;
    private Paths paths;

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),

                                    new Pose(90.078, 17.135)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(243))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.078, 17.135),

                                    new Pose(90.551, 33.635)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(243), Math.toRadians(360))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.551, 33.635),

                                    new Pose(136.365, 33.869)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(360))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.365, 33.869),

                                    new Pose(89.976, 17.128)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(241))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.976, 17.128),

                                    new Pose(92.201, 55.815)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(241), Math.toRadians(360))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(92.201, 55.815),

                                    new Pose(137.865, 55.808)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(360))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.865, 55.808),

                                    new Pose(90.044, 17.045)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(241))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.044, 17.045),

                                    new Pose(95.255, 35.703)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(236))

                    .build();
        }
    }





    public enum PathState {
        DRIVE_PATH1_SHOOT,
        SHOOT_PRELOADS,
        DRIVE_PATH2_INTAKE,
        DRIVE_PATH3_INTAKE,
        DRIVE_PATH4_SHOOT,
        SHOOT_CYCLE1,
        DRIVE_PATH5_INTAKE,
        DRIVE_PATH6_INTAKE,
        DRIVE_PATH7_SHOOT,
        SHOOT_CYCLE2,
        DRIVE_PATH8_PARK,
        IDLE
    }

    private PathState pathState = PathState.DRIVE_PATH1_SHOOT;
    private final Pose startPose = new Pose(88.0, 8.0, Math.toRadians(270)); // Red side start

    private void buildPaths() {
        paths = new Paths(follower);
    }

    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_PATH1_SHOOT:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path1, 0.7, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.SHOOT_PRELOADS);
                }
                break;

            case SHOOT_PRELOADS:
                IntakeStates.stop();
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(2200);
                    launcher.setAnglePosition(0.14);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH2_INTAKE);
                }
                break;

            case DRIVE_PATH2_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path2, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH3_INTAKE);
                }
                break;

            case DRIVE_PATH3_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999);
                    follower.followPath(paths.Path3, 0.3, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.DRIVE_PATH4_SHOOT);
                }
                break;

            case DRIVE_PATH4_SHOOT:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path4, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.SHOOT_CYCLE1);
                }
                break;

            case SHOOT_CYCLE1:
                IntakeStates.stop();
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(2150);
                    launcher.setAnglePosition(0.14);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH5_INTAKE);
                }
                break;

            case DRIVE_PATH5_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path5, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH6_INTAKE);
                }
                break;

            case DRIVE_PATH6_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path6, 0.3, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.7) {
                    setPathState(PathState.DRIVE_PATH7_SHOOT);
                }
                break;

            case DRIVE_PATH7_SHOOT:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path7, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.SHOOT_CYCLE2);
                }
                break;

            case SHOOT_CYCLE2:
                IntakeStates.stop();
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(2200);
                    launcher.setAnglePosition(0.14);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH8_PARK);
                }
                break;

            case DRIVE_PATH8_PARK:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path8, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                IntakeStates.stop();
                launcher.setTargetRpm(0);
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;
        flywheelStarted = false;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        launcher = new LauncherStates(hardwareMap);
        IntakeStates.init(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = PathState.DRIVE_PATH1_SHOOT;
        pathStarted = false;
        flywheelStarted = false;
    }

    @Override
    public void loop() {
        follower.update();
        launcher.update();
        IntakeStates.update();
        statePathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("path started", pathStarted);
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("launcher busy", launcher.isBusy());
        telemetry.addData("intake state", IntakeStates.intakeState);
        telemetry.addData("opMode time", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
