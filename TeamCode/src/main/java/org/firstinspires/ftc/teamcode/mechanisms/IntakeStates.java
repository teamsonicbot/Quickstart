package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeStates {


    public static DcMotor intakeMotor;


    public static CRServo leftIntake;
    public static CRServo rightIntake;

    // ===== Directions =====
    public static DcMotor.Direction intakeDirection = DcMotor.Direction.REVERSE;
    public static CRServo.Direction leftIntakeDirection = CRServo.Direction.FORWARD;
    public static CRServo.Direction rightIntakeDirection = CRServo.Direction.REVERSE;

    // ===== Powers =====
    public static double intakePower = 1.0;
    public static double reversePower = -1.0;

    // ===== Default Timer =====
    public static long intakeDefaultRunTime = 2000; // milliseconds

    // ===== State Machine =====
    public static int intakeState = 0; // 0 = off, 1 = forward, 2 = reverse
    public static ElapsedTime intakeTimer = new ElapsedTime();
    public static long intakeTargetTime = 0;

    // ===== Hardware Initialization =====
    public static void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        leftIntake = hwMap.get(CRServo.class, "Leftintake");
        rightIntake = hwMap.get(CRServo.class, "right intake");

        intakeMotor.setDirection(intakeDirection);
        leftIntake.setDirection(leftIntakeDirection);
        rightIntake.setDirection(rightIntakeDirection);
    }

    // ===== State Machine Update =====
    // Call this every loop in OpMode / PathBuilder
    public static void update() {
        switch (intakeState) {

            case 0: // OFF
                intakeMotor.setPower(0);
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                break;

            case 1: // FORWARD
                intakeMotor.setPower(intakePower);
                leftIntake.setPower(intakePower);
                rightIntake.setPower(intakePower);

                if (intakeTimer.milliseconds() >= intakeTargetTime) {
                    intakeState = 0; // automatically stop
                }
                break;

            case 2: // REVERSE
                intakeMotor.setPower(reversePower);
                leftIntake.setPower(reversePower);
                rightIntake.setPower(reversePower);

                if (intakeTimer.milliseconds() >= intakeTargetTime) {
                    intakeState = 0; // automatically stop
                }
                break;
        }
    }

    // ===== Helper Methods =====

    // Run forward for a specific number of milliseconds
    public static void runForward(long milliseconds) {
        intakeState = 1;
        intakeTargetTime = milliseconds;
        intakeTimer.reset();
    }

    // Run forward for default time
    public static void runForwardDefault() {
        runForward(intakeDefaultRunTime);
    }

    // Run reverse for a specific number of milliseconds
    public static void runReverse(long milliseconds) {
        intakeState = 2;
        intakeTargetTime = milliseconds;
        intakeTimer.reset();
    }

    // Stop immediately
    public static void stop() {
        intakeState = 0;
    }

}

