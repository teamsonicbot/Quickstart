package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "AprilTag Center (FINAL)", group = "Limelight")
@Disabled
public class Autoturnlime extends LinearOpMode {

    // ===== Drivetrain =====
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // ===== Limelight =====
    private Limelight3A limelight;

    // ===== Persistent State =====
    private double filteredTx = 0.0;
    private double smoothRx = 0.0;

    // ===== Tunables =====
    private static final double ALIGN_KP = 0.02;
    private static final double MAX_TURN_POWER = 0.15;
    private static final double MIN_TURN_POWER = 0.1;

    // Shooter-friendly settings
    private static final double TX_DEADBAND = 2.0;        // ignore tiny noise
    private static final double GOOD_ENOUGH_TX = 6.0;     // lock heading
    private static final double FAR_TA_THRESHOLD = 0.15;  // far range

    // Smoothing
    private static final double TX_FILTER_ALPHA = 0.25;
    private static final double RX_SMOOTH_ALPHA = 0.15;

    @Override
    public void runOpMode() {

        // ===== Hardware =====
        leftFront  = hardwareMap.get(DcMotor.class, "front left");
        leftBack   = hardwareMap.get(DcMotor.class, "back left");
        rightFront = hardwareMap.get(DcMotor.class, "front right");
        rightBack  = hardwareMap.get(DcMotor.class, "back right");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== Limelight =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        telemetry.addLine("Ready — Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ===== Read Limelight ONCE =====
            LLResult result = limelight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            double rawTx = 0.0;
            double ta = 0.0;

            if (hasTarget) {
                rawTx = result.getTx();
                ta = result.getTa();
            }

            // ===== Filter tx =====
            if (hasTarget) {
                filteredTx = (TX_FILTER_ALPHA * rawTx)
                        + ((1.0 - TX_FILTER_ALPHA) * filteredTx);
            } else {
                filteredTx = 0.0;
            }

            // ===== Auto-align logic =====
            double autoTurn = 0.0;

            if (gamepad1.a && hasTarget) {

                // Shooter lock — stop correcting when good enough
                if (Math.abs(filteredTx) < GOOD_ENOUGH_TX) {
                    autoTurn = 0.0;
                }
                // Normal alignment
                else if (Math.abs(filteredTx) > TX_DEADBAND) {

                    autoTurn = filteredTx * ALIGN_KP;

                    // Minimum power to move
                    if (autoTurn > 0) autoTurn += MIN_TURN_POWER;
                    else autoTurn -= MIN_TURN_POWER;

                    // Far-range confidence scaling
                    if (ta < FAR_TA_THRESHOLD) {
                        autoTurn *= 0.35;
                    }

                    autoTurn = Math.max(
                            -MAX_TURN_POWER,
                            Math.min(MAX_TURN_POWER, autoTurn)
                    );
                }
            }

            // ===== Smooth output =====
            smoothRx = (RX_SMOOTH_ALPHA * autoTurn)
                    + ((1.0 - RX_SMOOTH_ALPHA) * smoothRx);

            // ===== Driver controls =====
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.1;
            double rx =  gamepad1.right_stick_x + smoothRx;

            double denom = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            leftFront.setPower((y + x + rx) / denom);
            leftBack.setPower((y - x + rx) / denom);
            rightFront.setPower((y - x - rx) / denom);
            rightBack.setPower((y + x - rx) / denom);

            // ===== Telemetry =====
            telemetry.addData("Align Active", gamepad1.a);
            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("Raw tx", "%.2f", rawTx);
            telemetry.addData("Filtered tx", "%.2f", filteredTx);
            telemetry.addData("ta (confidence)", "%.3f", ta);
            telemetry.addData("Auto Turn", "%.2f", autoTurn);
            telemetry.update();
        }

        limelight.stop();
    }
}
