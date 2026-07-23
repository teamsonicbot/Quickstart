package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherStates {

    /* ================================
       HARDWARE
       ================================ */
    DcMotorEx leftLauncher;
    DcMotorEx rightLauncher;

    DcMotor intake;
    CRServo leftIntake;
    CRServo rightIntake;
    CRServo blueFan;

    Servo angleChanger;
    VoltageSensor battery;

    /* ================================
       TIMER
       ================================ */
    ElapsedTime stateTimer = new ElapsedTime();

    /* ================================
       STATES
       ================================ */
    enum FlywheelState {
        IDLE,
        SPIN_UP,
        FEEDING,
        STOPPING
    }

    FlywheelState flywheelState = FlywheelState.IDLE;

    /* ================================
       CONSTANTS
       ================================ */
    static final double TICKS_PER_REV = 28.0;

    static final double DEFAULT_TARGET_RPM = 2010;
    static final double DEFAULT_ANGLE_POSITION = 0.168;

    static final double RPM_TOLERANCE = 75;
    static final double MAX_SPINUP_TIME = 0.6;
    static final double FEED_TIME = 3.0;

    static final double kP = 0.0009;
    static final double kF = 19.5;
    static final double NOMINAL_VOLTAGE = 12.0;

    double targetRpm = DEFAULT_TARGET_RPM;
    double anglePosition = DEFAULT_ANGLE_POSITION;

    /* ================================
       CONSTRUCTOR
       ================================ */
    public LauncherStates(HardwareMap hardwareMap) {
        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntake  = hardwareMap.get(CRServo.class, "Leftintake");
        rightIntake = hardwareMap.get(CRServo.class, "right intake");
        blueFan     = hardwareMap.get(CRServo.class, "blue fan");
        angleChanger = hardwareMap.get(Servo.class, "Angle Changer");
        battery = hardwareMap.voltageSensor.iterator().next();

        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);
        blueFan.setDirection(DcMotor.Direction.REVERSE);
    }

    /* ================================
       PUBLIC API
       ================================ */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public void setAnglePosition(double position) {
        anglePosition = position;
    }

    public void useDefaultShot() {
        targetRpm = DEFAULT_TARGET_RPM;
        anglePosition = DEFAULT_ANGLE_POSITION;
    }

    public void requestShot() {
        if (flywheelState == FlywheelState.IDLE) {
            setState(FlywheelState.SPIN_UP);
        }
    }

    public void update() {
        switch (flywheelState) {
            case IDLE:
                stopAll();
                break;

            case SPIN_UP:
                spinUpFlywheels();
                if (atTargetRPM() || stateTimer.seconds() > MAX_SPINUP_TIME) {
                    setState(FlywheelState.FEEDING);
                }
                break;

            case FEEDING:
                feedArtifacts();
                if (stateTimer.seconds() > FEED_TIME) {
                    setState(FlywheelState.STOPPING);
                }
                break;

            case STOPPING:
                stopAll();
                setState(FlywheelState.IDLE);
                break;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    /* ================================
       STATE ACTIONS
       ================================ */
    void spinUpFlywheels() {
        applyPF();
        double ticksPerSecond = targetRpm * TICKS_PER_REV / 60.0;
        leftLauncher.setVelocity(ticksPerSecond);
        rightLauncher.setVelocity(ticksPerSecond);
        angleChanger.setPosition(anglePosition);
    }

    void feedArtifacts() {
        intake.setPower(1.0);
        leftIntake.setPower(1.0);
        rightIntake.setPower(1.0);
        blueFan.setPower(0.3);
    }

    void stopAll() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
        intake.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        blueFan.setPower(0);
    }

    /* ================================
       HELPERS
       ================================ */
    boolean atTargetRPM() {
        double currentRPM = leftLauncher.getVelocity() * 60.0 / TICKS_PER_REV;

        //return Math.abs(currentRPM - targetRpm) < RPM_TOLERANCE;
        return currentRPM >= targetRpm * 0.9;
    }

    void applyPF() {
        double voltage = battery.getVoltage();
        if (voltage <= 0) voltage = NOMINAL_VOLTAGE;
        double f = kF * (NOMINAL_VOLTAGE / voltage);
        leftLauncher.setVelocityPIDFCoefficients(kP, 0, 0, f);
        rightLauncher.setVelocityPIDFCoefficients(kP, 0, 0, f);
    }

    void setState(FlywheelState newState) {
        flywheelState = newState;
        stateTimer.reset();
    }
}
