package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name = "TELEOP_JAVA")
public class TeleOpsControl extends LinearOpMode {


    // ===== DRIVE =====
    private DcMotor backleft, backright, frontleft, frontright;


    // ===== SHOOTER =====
    private DcMotorEx leftLauncher, rightLauncher;
    private Servo angleChanger;


    // ===== INTAKE =====
    private DcMotor intake;
    private CRServo leftIntakeServo, rightIntakeServo;


    // ===== FAN =====
    private CRServo bluefan;


    // ===== VOLTAGE =====
    private VoltageSensor battery;


    // ===== CONSTANTS =====
    // goBILDA 6000 RPM Yellow Jacket: 28 encoder ticks per output shaft revolution [web:23][web:19]
    static final double TICKS_PER_REV = 28.0;


    // Presets (RPM + Angle)
    static final double CLOSE_RPM       = 2900;
    static final double MED_RPM         = 1800;
    static final double FAR_RPM         = 1950;
    static final double EXTRA_FAR_RPM   = 2300;


    static final double CLOSE_ANG       = 0.25;
    static final double MED_ANG         = 0.168;
    static final double FAR_ANG         = 0.15;
    static final double EXTRA_FAR_ANG   = 0.14;


    // ===== PF (P and F only) =====
    static final double kP              = 0.0009;
    static final double kF              = 19.5;      // tune per robot
    static final double NOMINAL_VOLTAGE = 12.0;


    // ===== VARIABLES =====
    double targetRpm   = 0.0;
    double targetAngle = CLOSE_ANG;
    String shooterMode = "Idle";


    @Override
    public void runOpMode() {


        // ===== MAP HARDWARE =====
        backleft   = hardwareMap.get(DcMotor.class, "back left");
        backright  = hardwareMap.get(DcMotor.class, "back right");
        frontleft  = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");


        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right launcher");


        intake = hardwareMap.get(DcMotor.class, "intake");


        angleChanger     = hardwareMap.get(Servo.class, "Angle Changer");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "Leftintake");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right intake");


        bluefan = hardwareMap.get(CRServo.class, "blue fan");


        battery = hardwareMap.voltageSensor.iterator().next();


        // ===== DIRECTIONS =====
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.FORWARD);


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


        // ===== INIT POSITIONS / STOP =====
        angleChanger.setPosition(CLOSE_ANG);
        stopAllFeeding();
        stopFlywheels();


        waitForStart();


        while (opModeIsActive()) {


            // ===== DRIVE (normalized mecanum) =====
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.1;
            double rx =  gamepad1.right_stick_x;


            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);


            frontleft.setPower((y + x + rx) / denom);
            backleft.setPower((y - x + rx) / denom);
            frontright.setPower((y - x - rx) / denom);
            backright.setPower((y + x - rx) / denom);


            // ===== SHOOTER PRESETS (Gamepad2 buttons HOLD) =====
            boolean shooting = false;


            if (gamepad2.x) {
                targetRpm   = EXTRA_FAR_RPM;
                targetAngle = EXTRA_FAR_ANG;
                shooterMode = "Extra Far";
                shooting    = true;
            } else if (gamepad2.b) {
                targetRpm   = MED_RPM;
                targetAngle = MED_ANG;
                shooterMode = "Medium";
                shooting    = true;
            } else if (gamepad2.y) {
                targetRpm   = CLOSE_RPM;
                targetAngle = CLOSE_ANG;
                shooterMode = "Close";
                shooting    = true;
            } else if (gamepad2.a) {
                targetRpm   = FAR_RPM;
                targetAngle = FAR_ANG;
                shooterMode = "Far";
                shooting    = true;
            }


            // ===== APPLY SHOOTING / STOP WHEN RELEASED =====
            if (shooting) {
                // Spin up flywheels to target velocity using PF control
                applyPF();
                double tps = rpmToTicksPerSecond(targetRpm);
                leftLauncher.setVelocity(tps);
                rightLauncher.setVelocity(tps);
                angleChanger.setPosition(targetAngle);


                // Only feed when average flywheel RPM is at >= 90% of target
                if (atTargetRPM()) {
                    runIntakes(1.0);
                    bluefan.setPower(1.0);
                } else {
                    // Let flywheels spin up, but do NOT feed rings yet
                    stopAllFeeding();
                }
            } else {
                // Shooter not requested -> stop flywheels and feeding
                stopFlywheels();
                stopAllFeeding();
                shooterMode = "Idle";
                targetRpm   = 0.0;


                // Optional manual intake when NOT shooting (bumpers)
                if (gamepad2.right_bumper) {
                    runIntakes(1.0);
                } else if (gamepad2.left_bumper) {
                    runIntakes(-1.0);
                } else {
                    stopIntakes();
                }
            }


            // ===== TELEMETRY =====
            telemetry.addData("Shooter Mode", shooterMode);
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Angle Position", targetAngle);
            telemetry.addData("Left Vel (tps)", leftLauncher.getVelocity());   // ticks/sec [web:14][web:11]
            telemetry.addData("Right Vel (tps)", rightLauncher.getVelocity());
            telemetry.addData("At >= 90% RPM?", atTargetRPM());
            telemetry.addData("Voltage", battery.getVoltage());
            telemetry.update();
        }
    }


    // ===== HELPERS =====


    // Convert RPM to ticks per second for setVelocity()
    double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }


    // Check if average flywheel RPM is at least 90% of target RPM
    boolean atTargetRPM() {
        // getVelocity() returns ticks per second [web:14][web:11]
        double leftTPS  = leftLauncher.getVelocity();
        double rightTPS = rightLauncher.getVelocity();
        double avgTPS   = (leftTPS + rightTPS) / 2.0;


        double avgRPM = avgTPS * 60.0 / TICKS_PER_REV;


        if (targetRpm <= 0) return false;
        return avgRPM >= targetRpm * 0.9;
    }


    // Apply P and voltage-compensated F
    void applyPF() {
        double voltage = battery.getVoltage();
        if (voltage <= 0) voltage = NOMINAL_VOLTAGE;


        double compensatedF = kF * (NOMINAL_VOLTAGE / voltage);


        leftLauncher.setVelocityPIDFCoefficients(kP, 0.0, 0.0, compensatedF);   // use velocity PIDF in RUN_USING_ENCODER [web:25][web:24]
        rightLauncher.setVelocityPIDFCoefficients(kP, 0.0, 0.0, compensatedF);
    }


    void runIntakes(double dir) {
        intake.setPower(dir);
        leftIntakeServo.setPower(dir);
        rightIntakeServo.setPower(dir);
    }


    void stopIntakes() {
        intake.setPower(0.0);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
    }


    void stopFlywheels() {
        leftLauncher.setVelocity(0.0);
        rightLauncher.setVelocity(0.0);
    }


    void stopAllFeeding() {
        stopIntakes();
        bluefan.setPower(0.0);
    }
}
