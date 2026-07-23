package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathConstraints;

public class Teleopconstants {
    // Smooth Pedro path (lower P-gains, no oscillation)
    public static final FollowerConstants pathFollowerConstants =
            new FollowerConstants()
                    .mass(10.85)
                    .forwardZeroPowerAcceleration(-35.09351151689762)
                    .lateralZeroPowerAcceleration(-64.2155023162645)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0.0, 0.01, 0.0))  // Lower P
                    .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0.0, 0.02, 0.01))        // Lower P
                    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.3, 0.0, 0.0001, 0.25, 0.0))
                    .centripetalScaling(0.0005);

    // Fast teleop tracking (high gains, snappy braking)
    public static final FollowerConstants teleopFollowerConstants =
            new FollowerConstants()
                    .mass(10.85)
                    .forwardZeroPowerAcceleration(-50.0)   // Stronger braking
                    .lateralZeroPowerAcceleration(-80.0)   // Stronger braking
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0.0, 0.02, 0.0))   // Higher P
                    .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0.0, 0.03, 0.01))        // Higher P
                    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.4, 0.0, 0.0001, 0.3, 0.0))
                    .centripetalScaling(0.0008);

    // Same tight path constraints
    public static final PathConstraints autoPathConstraints =
            new PathConstraints(0.99, 80, 3.0, 3.0);
}
