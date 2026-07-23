package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.LauncherStates;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeStates;

@Autonomous(name = "Blue1Auto", group = "Pedro", preselectTeleOp = "TELEOP_JAVA")
public class Blue1Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LauncherStates launcher;
    private boolean flywheelStarted = false;
    private boolean pathStarted = false;
    private Paths paths;

    // ---------- PATH DEFINITIONS ----------
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 125.000),

                                    new Pose(50.373, 96.580)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.373, 96.580),

                                    new Pose(58.097, 81.269)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.097, 81.269),

                                    new Pose(16.552, 80.938)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.552, 80.938),

                                    new Pose(50.331, 96.634)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-37))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.331, 96.634),

                                    new Pose(48.014, 59.297)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.014, 59.297),

                                    new Pose(10.593, 58.924)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.593, 58.924),
                                    new Pose(41.424, 56.817),
                                    new Pose(21.576, 84.459),
                                    new Pose(50.517, 96.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-40))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.517, 96.600),

                                    new Pose(31.945, 78.855)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-40))

                    .build();
        }
    }


    // ---------- STATE MACHINE ----------
    public enum PathState {
        DRIVE_PATH1_SHOOT,
        SHOOT_PRELOADS,
        DRIVE_PATH2_INTAKE,
        DRIVE_PATH3_INTAKE,
        DRIVE_PATH5_SHOOT,
        SHOOT_CYCLE1,
        DRIVE_PATH6_INTAKE,
        DRIVE_PATH7_INTAKE,
        DRIVE_PATH8_SHOOT,
        SHOOT_CYCLE2,
        DRIVE_PATH9_PARK,
        IDLE
    }

    private PathState pathState = PathState.DRIVE_PATH1_SHOOT;
    private final Pose startPose = new Pose(22.0, 125.0, Math.toRadians(-37));

    private void buildPaths() {
        paths = new Paths(follower);
    }

    // ---------- STATE UPDATE ----------
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
                    launcher.setTargetRpm(1800);
                    launcher.setAnglePosition(0.15);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH2_INTAKE);
                }
                break;

            case DRIVE_PATH2_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path2, 0.7, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH3_INTAKE);
                }
                break;

            case DRIVE_PATH3_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path3, 0.5, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(PathState.DRIVE_PATH5_SHOOT);
                }
                break;

            case DRIVE_PATH5_SHOOT:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path5, 0.8, true);
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
                    launcher.setTargetRpm(1830);
                    launcher.setAnglePosition(0.15);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH6_INTAKE);
                }
                break;

            case DRIVE_PATH6_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path6, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH7_INTAKE);
                }
                break;

            case DRIVE_PATH7_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path7, 0.5, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    setPathState(PathState.DRIVE_PATH8_SHOOT);
                }
                break;

            case DRIVE_PATH8_SHOOT:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path8, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.SHOOT_CYCLE2);
                }
                break;

            case SHOOT_CYCLE2:
                IntakeStates.stop();
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(1830);
                    launcher.setAnglePosition(0.156);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH9_PARK);
                }
                break;

            case DRIVE_PATH9_PARK:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path9, 0.8, true);
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

    // ---------- OPMODE LIFECYCLE ----------
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
        pathState = PathState.DRIVE_PATH1_SHOOT;   // ensure fresh state at start
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
        telemetry.addData("opmode time", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
