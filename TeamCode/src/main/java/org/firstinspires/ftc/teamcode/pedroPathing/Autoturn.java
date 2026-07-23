package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "TELEOP_JAVA_LIMELIGHT (FIXED)", group = "Limelight")
public class Autoturn extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor backleft, backright, frontleft, frontright;

    // ================= LIMELIGHT =================
    private Limelight3A limelight;

    private double filteredTx = 0.0;
    private double smoothRx = 0.0;
    private int turnDirection = 0;
    private boolean headingLocked = false;

    // ===== APRILTAG CONSTANTS =====
    private static final double ALIGN_KP = 0.005;
    private static final double MAX_TURN_POWER = 0.30;

    private static final double FAR_TA = 0.14;
    private static final double FAR_TURN_POWER = 0.06;

    private static final double TX_DEADBAND_FAR   = 1.0;
    private static final double TX_DEADBAND_CLOSE = 4.0;

    // 🔧 NEW FIX CONSTANTS
    private static final double TX_LOCK_THRESHOLD = 2.0;   // lock only when REALLY centered
    private static final double MIN_VALID_TA = 0.03;       // ignore sideways tag detections

    private static final double TX_FILTER_ALPHA = 0.25;
    private static final double RX_SMOOTH_ALPHA = 0.20;

    // ================= SHOOTER =================
    private DcMotorEx leftLauncher, rightLauncher;
    private Servo angleChanger;

    // ================= INTAKE =================
    private DcMotor intake;
    private CRServo leftIntakeServo, rightIntakeServo;

    // ================= FAN =================
    private CRServo bluefan;

    // ================= VOLTAGE =================
    private VoltageSensor battery;

    // ================= SHOOTER CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double CLOSE_RPM = 2900;
    static final double MED_RPM   = 1800;
    static final double FAR_RPM   = 1950;
    static final double EXTRA_FAR_RPM = 2300;

    static final double CLOSE_ANG = 0.25;
    static final double MED_ANG   = 0.168;
    static final double FAR_ANG   = 0.15;
    static final double EXTRA_FAR_ANG = 0.14;

    static final double kP = 0.0009;
    static final double kF = 19.5;
    static final double NOMINAL_VOLTAGE = 12.0;

    double targetRpm = 0.0;
    double targetAngle = CLOSE_ANG;
    String shooterMode = "Idle";

    @Override
    public void runOpMode() {

        // ===== HARDWARE MAP =====
        backleft   = hardwareMap.get(DcMotor.class, "back left");
        backright  = hardwareMap.get(DcMotor.class, "back right");
        frontleft  = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");

        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right launcher");

        intake = hardwareMap.get(DcMotor.class, "intake");

        angleChanger = hardwareMap.get(Servo.class, "Angle Changer");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "Leftintake");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right intake");

        bluefan = hardwareMap.get(CRServo.class, "blue fan");

        battery = hardwareMap.voltageSensor.iterator().next();

        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // ===== DIRECTIONS =====
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);

        bluefan.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);

        // ===== MODES =====
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== BRAKES =====
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleChanger.setPosition(CLOSE_ANG);
        stopAllFeeding();
        stopFlywheels();

        waitForStart();

        while (opModeIsActive()) {

            // ================= APRILTAG AUTO TURN =================
            LLResult result = limelight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            double rawTx = hasTarget ? result.getTx() : 0.0;
            double ta    = hasTarget ? result.getTa() : 0.0;

            filteredTx = hasTarget
                    ? TX_FILTER_ALPHA * rawTx + (1 - TX_FILTER_ALPHA) * filteredTx
                    : 0.0;

            double autoTurn = 0.0;

            // 🔧 FIX #2: require tag to actually be in front
            if (gamepad1.a && hasTarget && ta > MIN_VALID_TA) {

                boolean isFar = ta < FAR_TA;
                double deadband = isFar ? TX_DEADBAND_FAR : TX_DEADBAND_CLOSE;

                if (!headingLocked) {

                    if (Math.abs(filteredTx) > deadband) {

                        if (turnDirection == 0)
                            turnDirection = (int) Math.signum(filteredTx);

                        autoTurn = turnDirection *
                                (isFar ? FAR_TURN_POWER
                                        : Math.abs(filteredTx) * ALIGN_KP);

                    } else {
                        // 🔧 FIX #1: stop turning first, lock later
                        autoTurn = 0.0;

                        if (Math.abs(filteredTx) < TX_LOCK_THRESHOLD) {
                            headingLocked = true;
                            turnDirection = 0;
                        }
                    }
                }

                autoTurn = Math.max(
                        -MAX_TURN_POWER,
                        Math.min(MAX_TURN_POWER, autoTurn)
                );

            } else {
                filteredTx = 0.0;
                turnDirection = 0;
                headingLocked = false;
            }

            smoothRx = RX_SMOOTH_ALPHA * autoTurn
                    + (1 - RX_SMOOTH_ALPHA) * smoothRx;

            // ================= DRIVE =================
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.1;
            double rx =  gamepad1.right_stick_x + smoothRx;

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            frontleft.setPower((y + x + rx) / denom);
            backleft.setPower((y - x + rx) / denom);
            frontright.setPower((y - x - rx) / denom);
            backright.setPower((y + x - rx) / denom);

            // ================= SHOOTER (UNCHANGED) =================
            boolean shooting = false;

            if (gamepad2.x) {
                targetRpm = EXTRA_FAR_RPM;
                targetAngle = EXTRA_FAR_ANG;
                shooterMode = "Extra Far";
                shooting = true;
            } else if (gamepad2.b) {
                targetRpm = MED_RPM;
                targetAngle = MED_ANG;
                shooterMode = "Medium";
                shooting = true;
            } else if (gamepad2.y) {
                targetRpm = CLOSE_RPM;
                targetAngle = CLOSE_ANG;
                shooterMode = "Close";
                shooting = true;
            } else if (gamepad2.a) {
                targetRpm = FAR_RPM;
                targetAngle = FAR_ANG;
                shooterMode = "Far";
                shooting = true;
            }

            if (shooting) {
                applyPF();
                double tps = rpmToTicksPerSecond(targetRpm);
                leftLauncher.setVelocity(tps);
                rightLauncher.setVelocity(tps);
                angleChanger.setPosition(targetAngle);
                runIntakes(1.0);
                bluefan.setPower(1.0);
            } else {
                stopFlywheels();
                stopAllFeeding();
                shooterMode = "Idle";
            }

            telemetry.addData("TX", filteredTx);
            telemetry.addData("TA", ta);
            telemetry.addData("Locked", headingLocked);
            telemetry.addData("AutoTurn", smoothRx);
            telemetry.addData("Shooter Mode", shooterMode);
            telemetry.update();
        }

        limelight.stop();
    }

    // ================= HELPERS =================
    double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    void applyPF() {
        double voltage = battery.getVoltage();
        if (voltage <= 0) voltage = NOMINAL_VOLTAGE;
        double compensatedF = kF * (NOMINAL_VOLTAGE / voltage);
        leftLauncher.setVelocityPIDFCoefficients(kP, 0, 0, compensatedF);
        rightLauncher.setVelocityPIDFCoefficients(kP, 0, 0, compensatedF);
    }

    void runIntakes(double dir) {
        intake.setPower(dir);
        leftIntakeServo.setPower(dir);
        rightIntakeServo.setPower(dir);
    }

    void stopIntakes() {
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    void stopFlywheels() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
    }

    void stopAllFeeding() {
        stopIntakes();
        bluefan.setPower(0);
    }
}
