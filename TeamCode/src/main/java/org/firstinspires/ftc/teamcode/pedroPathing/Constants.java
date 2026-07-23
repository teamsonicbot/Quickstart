// Defines the folder/package location where this file lives inside your project
package org.firstinspires.ftc.teamcode.pedroPathing;

// Imports the specialized PIDF tuning tool that filters out sensor noise
import com.pedropathing.control.FilteredPIDFCoefficients;
// Imports the standard mathematical tool used to calculate positional corrections
import com.pedropathing.control.PIDFCoefficients;
// Imports the core engine object responsible for driving the robot along paths
import com.pedropathing.follower.Follower;
// Imports the structure used to configure physical robot properties (like mass)
import com.pedropathing.follower.FollowerConstants;
// Imports the setup tool used to piece together your final Follower engine
import com.pedropathing.ftc.FollowerBuilder;
// Imports the structure used to map and control Mecanum drive wheels
import com.pedropathing.ftc.drivetrains.MecanumConstants;
// Imports the configuration structure specifically for goBILDA Pinpoint tracking hardware
import com.pedropathing.ftc.localization.constants.PinpointConstants;
// Imports the tool used to set temporary or permanent top speed limits
import com.pedropathing.paths.PathConstraints;
// Imports the physical software driver needed to talk to the goBILDA Pinpoint computer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
// Imports the simple direction settings (FORWARD/REVERSE) for electronic motors
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// Imports the FTC master list that links code names to physical hub ports
import com.qualcomm.robotcore.hardware.HardwareMap;
// Imports standard unit measurements so the computer understands inches vs millimeters
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Declares the public class named Constants so other programs can read these variables
public class Constants {

    // Creates a globally accessible configuration block for general robot physics and movement
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Sets the weight of the robot to 10.85 kilograms (approx. 24 lbs) for deceleration math
            .mass(10.85)
            // Calculated deceleration rate when coasting forward with zero motor power (from tuning)
            .forwardZeroPowerAcceleration(-35.09351151689762)
            // Calculated deceleration rate when coasting sideways with zero motor power (from tuning)
            .lateralZeroPowerAcceleration(-64.2155023162645)
            // Slightly calmer translational & heading PID to reduce oscillation
            // Configures positional corrections (X and Y axis error): Proportional=0.12, Integral=0, Derivative=0.008, Feedforward=0
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0 , 0.008, 0))
            // Configures rotational corrections (angle error): Proportional=0.9, Integral=0, Derivative=0.012, Feedforward=0.01
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.012, 0.01))
            // Configures raw motor power corrections, using a low-pass filter factor of 0.2 to smooth out sudden voltage spikes
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2, 0.0, 0.0001, 0.2, 0))
            // Dynamically slows down the robot in sharp corners to prevent sliding out or tipping over
            .centripetalScaling(0.0005);

    // Creates a globally accessible configuration block for your Mecanum wheel layout
    public static MecanumConstants driveConstants = new MecanumConstants()
            // Limits the absolute maximum motor power output to 100% cap
            .maxPower(1)
            // Links the front-right motor variable to the exact name typed in your Driver Station App
            .rightFrontMotorName("front right")
            // Links the back-right motor variable to the exact name typed in your Driver Station App
            .rightRearMotorName("back right")
            // Links the back-left motor variable to the exact name typed in your Driver Station App
            .leftRearMotorName("back left")
            // Links the front-left motor variable to the exact name typed in your Driver Station App
            .leftFrontMotorName("front left")
            // Flips the front-left motor direction so driving forward doesn't spin the robot in circles
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Flips the back-left motor direction so driving forward doesn't spin the robot in circles
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Keeps the front-right motor spinning in its natural forward direction
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Keeps the back-right motor spinning in its natural forward direction
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Calibrated top physical speed achieved when driving straight forward (inches per second)
            .xVelocity(56.479478730930126)
            // Calibrated top physical speed achieved when strafing directly sideways (inches per second)
            .yVelocity(46.99349615142103);

    // ORIGINAL constraints for AUTONOMOUS – keep using this in auto
    // Sets the default speed envelope: 99% power, 100 in/s max speed, 2 rad/s turn speed, structural path structural factor of 1
    public static PathConstraints pathConstraints = new PathConstraints( 0.99, 100, 2, 1 );

    // Creates a globally accessible configuration block for the goBILDA Pinpoint tracker
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // The distance (in inches) the forward-facing encoder pod sits away from the center center-line of the robot
            .forwardPodY(0.314961)
            // The distance (in inches) the sideways-facing encoder pod sits away from the center center-line of the robot
            .strafePodX(1.88976)
            // Declares that the measurements provided above are written explicitly in inches
            .distanceUnit(DistanceUnit.INCH)
            // Links the Pinpoint hardware variale to the exact name typed in your Driver Station App config
            .hardwareMapName("odo")
            // Identifies the precise hardware model (4-bar style pods) to automatically apply correct internal math conversions
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // Dictates that counting numbers upwards moves the robot forward (rather than backward)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            // Dictates that counting numbers upwards moves the robot to the right (rather than left)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // A builder method that pulls all the chunks of information above together to assemble a ready-to-run drive engine
    public static Follower createFollower(HardwareMap hardwareMap) {
        // Starts the building process by feeding the general physics properties and the current hardware list
        return new FollowerBuilder(followerConstants, hardwareMap)
                // Plugs in your goBILDA Pinpoint position configuration data
                .pinpointLocalizer(localizerConstants)
                // This is still the DEFAULT constraints; auto can override or use directly
                // Plugs in your default speed and velocity caps
                .pathConstraints(pathConstraints)
                // Plugs in your hardware motor name maps and motor directional properties
                .mecanumDrivetrain(driveConstants)
                // Finalizes assembly and delivers a fully operational path-following object to your team code
                .build();
    }
}
