package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherStates;

@Autonomous
@Disabled
public class SampleBlue1_AllArtifacts extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LauncherStates launcher;
    private boolean flywheelStarted = false;
    private boolean pathStarted = false;
    private Paths paths;

    // ✅ NEW COORDINATES INTEGRATED
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 125.000),
                                    new Pose(46.897, 95.090)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))
                    .build();
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.897, 95.090),
                                    new Pose(49.821, 59.090)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .build();
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.821, 59.090),
                                    new Pose(19.365, 59.255)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();
            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.365, 59.255),
                                    new Pose(26.576, 62.966),
                                    new Pose(22.200, 63.972)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();
            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(22.200, 63.972),
                                    new Pose(51.963, 65.886),
                                    new Pose(53.793, 84.083)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-45))
                    .build();
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.793, 84.083),
                                    new Pose(47.876, 84.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .build();
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.876, 84.131),
                                    new Pose(22.745, 84.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.745, 84.131),
                                    new Pose(46.676, 94.841)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-45))
                    .build();
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.676, 94.841),
                                    new Pose(49.862, 35.952)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .build();
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.862, 35.952),
                                    new Pose(20.875, 35.911)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();
            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.875, 35.911),
                                    new Pose(53.793, 83.931)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-45))
                    .build();
            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.793, 83.931),
                                    new Pose(43.876, 73.290)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-45))
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
            // 🔥 PreLOad position
            case DRIVE_PATH1:
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path1, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.SHOOT_PRELOADS);
                }
                break;

            case SHOOT_PRELOADS:
                IntakeStates.stop();  // ✅ STOP MAIN INTAKE

                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(1899);
                    launcher.setAnglePosition(0.17);
                    launcher.requestShot();  // Shoot all 3 preloads
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.25) {
                    setPathState(PathState.DRIVE_PATH2);
                }
                break;

            // 🔥 CYCLE 1: Path2(0.8) → Path3(0.6 intake) → Path4(0.8 close gate) → Path5(0.8 shoot)
            case DRIVE_PATH2: // Goes toward positon of intake set 1(middle)
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path2, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH3_INTAKE);
                }
                break;

            case DRIVE_PATH3_INTAKE: //Intakes the intake set 1(middle)
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path3, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.DRIVE_PATH4_CLOSE_GATE);
                }
                break;

            case DRIVE_PATH4_CLOSE_GATE: //opens the gate
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path4, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;

            case DRIVE_PATH5: //goes toward shooting position
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();  // ✅ STOP MAIN INTAKE BEFORE SHOOT
                    follower.followPath(paths.Path5, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.SHOOT_CYCLE1);
                }
                break;

            case SHOOT_CYCLE1:
                IntakeStates.stop();  // Shoots the 3 artifacts
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

            // 🔥 Rotates to set up for intake
            case DRIVE_PATH6_ROTATE:
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path6, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.6) {  // Slight pause
                    setPathState(PathState.DRIVE_PATH7_INTAKE);
                }
                break;

            case DRIVE_PATH7_INTAKE: //Path 7 intakes the first set of balls(second)
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path7, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.DRIVE_PATH8_SHOOTPOS);
                }
                break;

            case DRIVE_PATH8_SHOOTPOS: //goes toward shooting position to shoot those artifacts
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path8, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.SHOOT_CYCLE2);
                }
                break;

            case SHOOT_CYCLE2:
                IntakeStates.stop();  // ✅ STOP MAIN INTAKE
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(1800);
                    launcher.setAnglePosition(0.168);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.25) {
                    setPathState(PathState.DRIVE_PATH9_POSITION);
                }
                break;

            // 🔥 CYCLE 3: Path9(0.8 position) → Path10(0.5 intake) → Path11(0.8 shoot pos)
            case DRIVE_PATH9_POSITION: // Goes toawrds next artifacts set last
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path9, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH10_INTAKE);
                }
                break;

            case DRIVE_PATH10_INTAKE: // intakes hte last artifacts
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.runForward(999999L);
                    follower.followPath(paths.Path10, 0.8, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.DRIVE_PATH11_SHOOTPOS);
                }
                break;

            case DRIVE_PATH11_SHOOTPOS: //goes to the shooting position
                if (!follower.isBusy() && !pathStarted) {
                    follower.followPath(paths.Path11, 1, true);
                    pathTimer.resetTimer();
                    pathStarted = true;
                }
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.SHOOT_CYCLE3);
                }
                break;

            case SHOOT_CYCLE3:
                IntakeStates.stop();  // ✅ STOP MAIN INTAKE
                launcher.update();
                if (!flywheelStarted) {
                    launcher.setTargetRpm(1960);
                    launcher.setAnglePosition(0.168);
                    launcher.requestShot();
                    flywheelStarted = true;
                }
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.25) {
                    setPathState(PathState.DRIVE_PATH12_PARK);
                }
                break;

            // 🔥 FINAL PARK
            case DRIVE_PATH12_PARK: //leaves the triangle zone
                if (!follower.isBusy() && !pathStarted) {
                    IntakeStates.stop();
                    follower.followPath(paths.Path12, 1.0, true);
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
