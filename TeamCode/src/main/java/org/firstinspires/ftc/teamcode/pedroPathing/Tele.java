package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.*;
import com.pedropathing.ftc.FollowerBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.Teleopconstants;

@TeleOp(name = "TELE-PEDRO")
public class Tele extends LinearOpMode {

    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private DcMotorEx leftLauncher, rightLauncher;
    private Servo angleChanger;
    private DcMotor intake;
    private CRServo leftIntakeServo, rightIntakeServo, blueFan;
    private VoltageSensor battery;

    private Follower follower;
    private boolean automatedDrive = false;
    private boolean rightBumperHeld = false;

    static final Pose TARGET_POSE = new Pose(57.0, 17.3, Math.toRadians(-69.0));
    static final Pose START_POSE = new Pose(48.745, 35.703, Math.toRadians(-69.0));  // Auto end pose
    static final double TICKS_PER_REV = 28.0;
    static final double CLOSE_RPM = 2900, MED_RPM = 1800, FAR_RPM = 1950, EXTRA_FAR_RPM = 2300;
    static final double CLOSE_ANG = 0.25, MED_ANG = 0.168, FAR_ANG = 0.15, EXTRA_FAR_ANG = 0.14;
    static final double kP = 0.0009, kF = 19.5, NOMINAL_VOLTAGE = 12.0;

    double targetRpm = 0, targetAngle = CLOSE_ANG;
    String shooterMode = "Idle";

    @Override
    public void runOpMode() {
        // YOUR ORIGINAL HARDWARE (unchanged)
        backLeft = hardwareMap.get(DcMotor.class, "back left");
        backRight = hardwareMap.get(DcMotor.class, "back right");
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        angleChanger = hardwareMap.get(Servo.class, "Angle Changer");
        leftIntakeServo = hardwareMap.get(CRServo.class, "Leftintake");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right intake");
        blueFan = hardwareMap.get(CRServo.class, "blue fan");
        battery = hardwareMap.voltageSensor.iterator().next();

        // YOUR ORIGINAL MOTOR CONFIG (unchanged)
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        blueFan.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleChanger.setPosition(CLOSE_ANG);
        stopAllFeeding();
        stopFlywheels();

        // FAST TELEOP FOLLOWER (high brake, snappy)
        follower = new FollowerBuilder(Teleopconstants.teleopFollowerConstants, hardwareMap).build();
        follower.setPose(START_POSE);  // Set exact autonomous end position

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // RIGHT BUMPER: Press=start smooth path, Release=cancel
            boolean rightBumper = gamepad1.right_bumper;

            if (rightBumper && !rightBumperHeld && !automatedDrive) {
                follower.setConstants(Teleopconstants.pathFollowerConstants);  // Switch to smooth
                PathChain chain = follower.pathBuilder()
                        .setConstraints(Teleopconstants.autoPathConstraints)
                        .addPath(new Path(new BezierLine(follower::getPose, TARGET_POSE)))
                        .build();
                follower.followPath(chain);
                automatedDrive = true;
            }

            if (automatedDrive && (!rightBumper || !follower.isBusy())) {
                stopDriveMotors();
                follower.setConstants(Teleopconstants.teleopFollowerConstants);  // Back to fast
                automatedDrive = false;
            }

            rightBumperHeld = rightBumper;

            // SNAPPY MANUAL DRIVE (unchanged - stays fast)
            if (!automatedDrive) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;
                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.01);

                frontLeft.setPower((y + x + rx) / denom);
                backLeft.setPower((y - x + rx) / denom);
                frontRight.setPower((y - x - rx) / denom);
                backRight.setPower((y + x - rx) / denom);
            }

            handleShooter();
            updateTelemetry();
        }
    }

    // ALL YOUR SHOOTER CODE (unchanged)
    private void handleShooter() {
        boolean shooting = false;
        if (gamepad2.x) { targetRpm = EXTRA_FAR_RPM; targetAngle = EXTRA_FAR_ANG; shooterMode = "Extra Far"; shooting = true; }
        else if (gamepad2.b) { targetRpm = MED_RPM; targetAngle = MED_ANG; shooterMode = "Medium"; shooting = true; }
        else if (gamepad2.y) { targetRpm = CLOSE_RPM; targetAngle = CLOSE_ANG; shooterMode = "Close"; shooting = true; }
        else if (gamepad2.a) { targetRpm = FAR_RPM; targetAngle = FAR_ANG; shooterMode = "Far"; shooting = true; }

        if (shooting) {
            applyPF();
            double tps = rpmToTicksPerSecond(targetRpm);
            leftLauncher.setVelocity(tps);
            rightLauncher.setVelocity(tps);
            angleChanger.setPosition(targetAngle);
            if (atTargetRPM()) {
                runIntakes(1.0);
                blueFan.setPower(1.0);
            } else {
                stopAllFeeding();
            }
        } else {
            stopFlywheels();
            stopAllFeeding();
            shooterMode = "Idle";
            targetRpm = 0;
            if (gamepad2.right_bumper) runIntakes(1.0);
            else if (gamepad2.left_bumper) runIntakes(-1.0);
            else stopIntakes();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Mode", automatedDrive ? "AUTO" : "MANUAL");
        telemetry.addData("Pose X", "%.1f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("RB", gamepad1.right_bumper ? "HELD" : "OFF");
        telemetry.addData("Shooter", shooterMode);
        telemetry.addData("Constants", automatedDrive ? "PATH" : "TELEOP");
        telemetry.update();
    }

    // YOUR HELPERS (unchanged)
    private double rpmToTicksPerSecond(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    private double ticksPerSecondToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }
    private boolean atTargetRPM() {
        double avgRPM = ticksPerSecondToRpm((leftLauncher.getVelocity() + rightLauncher.getVelocity()) / 2.0);
        return targetRpm > 0 && avgRPM >= targetRpm * 0.9;
    }
    private void applyPF() {
        double voltage = battery.getVoltage() > 0 ? battery.getVoltage() : NOMINAL_VOLTAGE;
        double f = kF * (NOMINAL_VOLTAGE / voltage);
        leftLauncher.setVelocityPIDFCoefficients(kP, 0, 0, f);
        rightLauncher.setVelocityPIDFCoefficients(kP, 0, 0, f);
    }
    private void runIntakes(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }
    private void stopIntakes() {
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
    private void stopFlywheels() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
    }
    private void stopAllFeeding() {
        stopIntakes();
        blueFan.setPower(0);
    }
    private void stopDriveMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
}
