package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherStates;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeStates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Disabled
public class SAMPLE extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LauncherStates launcher;
    private boolean flywheelStarted = false;
    private boolean pathStarted = false;
    private Paths paths;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 125.000),
                                    new Pose(46.897, 98.897)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))
                    .setBrakingStrength(0.3)     // ✅ SMOOTH BRAKING
                    .setBrakingStart(0.85)       // ✅ EARLY DECEL
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.897, 98.897),
                                    new Pose(51.476, 57.766)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.476, 57.766),
                                    new Pose(19.531, 58.262)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.531, 58.262),
                                    new Pose(29.576, 62.966),
                                    new Pose(22.200, 63.972)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(22.200, 63.972),
                                    new Pose(51.963, 65.886),
                                    new Pose(58.807, 84.545)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-45))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.807, 84.545),
                                    new Pose(48.207, 84.628)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.207, 84.628),
                                    new Pose(22.745, 84.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.745, 84.131),
                                    new Pose(46.938, 98.786)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-45))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.938, 98.786),
                                    new Pose(52.841, 34.814)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.841, 34.814),
                                    new Pose(21.703, 34.752)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.703, 34.752),
                                    new Pose(58.724, 84.434)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-45))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.724, 84.434),
                                    new Pose(43.069, 68.145)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-45))
                    .setBrakingStrength(0.3)
                    .setBrakingStart(0.85)
                    .build();
        }
    }

    public enum PathState {
        DRIVE_PATH1, SHOOT_PRELOADS,
        DRIVE_PATH2, DRIVE_PATH3_INTAKE, DRIVE_PATH4_CLOSE_GATE,
        DRIVE_PATH5, SHOOT_CYCLE1,
        DRIVE_PATH6_ROTATE, DRIVE_PATH7_INTAKE, DRIVE_PATH8_SHOOTPOS,
        SHOOT_CYCLE2,
        DRIVE_PATH9_POSITION, DRIVE_PATH10_INTAKE, DRIVE_PATH11_SHOOTPOS,
        SHOOT_CYCLE3,
        DRIVE_PATH12_PARK,
        IDLE
    }

    private PathState pathState = PathState.DRIVE_PATH1;
    private final Pose startPose = new Pose(22.0, 125.0, Math.toRadians(-37));

    public void buildPaths() {
        paths = new Paths(follower);
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_PATH1:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path1, 0.9, true);  // ✅ LOWER SPEED
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
                    launcher.setTargetRpm(2000);
                    launcher.setAnglePosition(0.175);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.DRIVE_PATH2);
                }
                break;

            case DRIVE_PATH2:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path2, 0.9, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
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
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.DRIVE_PATH4_CLOSE_GATE);
                }
                break;

            case DRIVE_PATH4_CLOSE_GATE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path4, 0.9, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;

            case DRIVE_PATH5:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path5, 0.9, true);
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
                    launcher.setTargetRpm(2100);
                    launcher.setAnglePosition(0.168);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.DRIVE_PATH6_ROTATE);
                }
                break;

            case DRIVE_PATH6_ROTATE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path6, 0.9, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.6) {
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
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.DRIVE_PATH8_SHOOTPOS);
                }
                break;

            case DRIVE_PATH8_SHOOTPOS:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path8, 0.9, true);
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
                    launcher.setTargetRpm(1800);
                    launcher.setAnglePosition(0.168);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.DRIVE_PATH9_POSITION);
                }
                break;

            case DRIVE_PATH9_POSITION:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path9, 0.9, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.DRIVE_PATH10_INTAKE);
                }
                break;

            case DRIVE_PATH10_INTAKE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path10, 0.5, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.DRIVE_PATH11_SHOOTPOS);
                }
                break;

            case DRIVE_PATH11_SHOOTPOS:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path11, 0.9, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.SHOOT_CYCLE3);
                }
                break;

            case SHOOT_CYCLE3:
                IntakeStates.stop();
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(1960);
                    launcher.setAnglePosition(0.168);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.DRIVE_PATH12_PARK);
                }
                break;

            case DRIVE_PATH12_PARK:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path12, 0.9, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                IntakeStates.stop();
                launcher.setTargetRpm(0);
                break;
        }
    }

    public void setPathState(PathState newState) {
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
