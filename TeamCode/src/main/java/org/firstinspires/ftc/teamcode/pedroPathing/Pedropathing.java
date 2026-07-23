package org.firstinspires.ftc.teamcode.pedroPathing; // Defines the directory where this file resides

// === IMPORTING EXTERNAL LIBRARIES & DEPENDENCIES ===
import com.pedropathing.follower.Follower; // Imports the main engine that controls wheel vectors
import com.pedropathing.paths.PathChain; // Imports the tool used to link multiple paths together seamlessly
import com.pedropathing.util.Timer; // Imports PedroPathing's high-precision stopwatch tool
import com.pedropathing.geometry.BezierLine; // Imports the math blueprint for driving straight lines
import com.pedropathing.geometry.BezierCurve; // Imports the math blueprint for driving custom curved paths
import com.pedropathing.geometry.Pose; // Imports the coordinate tool that tracks (X, Y, Heading)
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Imports the registration tag for Autonomous mode
import com.qualcomm.robotcore.eventloop.opmode.OpMode; // Imports the base continuous loop program template
import org.firstinspires.ftc.teamcode.mechanisms.LauncherStates; // Your team's custom launcher manager class
import org.firstinspires.ftc.teamcode.mechanisms.IntakeStates;   // IMPORTED: Your team's custom intake manager class

// Registers the program on the Driver Station screen with specific tags and preselects TeleOp
@Autonomous(name = "Pedropathing Guide")
public class Pedropathing extends OpMode { // Opens the main class container for your autonomous program

    // === 1. HARDWARE & TRACKING VARIABLE DECLARATIONS ===
    private Follower follower;         // Controls the 4 wheel motors via Constants.java
    private LauncherStates launcher;   // Your team's launcher subsystem

    // Gatekeepers: Controls when actions are allowed to run
    private boolean pathStarted = false; // Interlock gate: prevents code from constantly refiring a path command
    private boolean flywheelStarted = false; // Interlock gate: prevents code from constantly restarting motor power
    private boolean intakeStarted = false; // NEW GATEKEEPER: Prevents code from spamming intake motor commands repeatedly mid-drive

    // Our non-blocking stopwatch timers
    private Timer pathTimer; // High-precision stopwatch to manage driving path step durations
    private Timer opModeTimer; // High-precision stopwatch to track overall autonomous runtime

    // 2. THE PATH BINDER VARIABLE
    private Paths paths; // Container variable to store our compiled visualizer field path coordinates

    // 3. THE PATHS BINDER BLUEPRINT (The nested class)
    public static class Paths { // Opens the sub-box where all visualizer lines are pre-built at initialization
        public PathChain Path1; // Storage slot for your first straight line path
        public PathChain Path2; // Storage slot for your second path (the first curved bend)
        public PathChain Path3; // Storage slot for your third path (the final curved target approach)

        public Paths(Follower follower) { // Constructor method: builds the paths whenever called

            // === VISUALIZER LINES AND CURVES GET BUILT RIGHT HERE! ===

            // building Path 1: Straight line up the field
            Path1 = follower.pathBuilder() // Initiates building path segment 1
                    .addPath(
                            new BezierLine( // Tells the computer this path segment is perfectly straight
                                    new Pose(56.000, 8.000),   // Starting coordinate
                                    new Pose(56.000, 36.000)   // Ending coordinate
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)) // Rotate while moving heading change
                    .build(); // Saves this path to page "Path1"

            // building Path 2: First custom curve from your visualizer (Uses 1 control point)
            Path2 = follower.pathBuilder() // Initiates building path segment 2
                    .addPath(
                            new BezierCurve( // Tells the computer this segment bends using a control point curvature
                                    new Pose(56.000, 36.000),  // Must match ending point of Path 1 : where the path starts
                                    new Pose(44.999, 35.000),  // Control Point : Bend
                                    new Pose(30.000, 55.000)   // Destination of this curve
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270)) // Heading change 180 to 270
                    .build(); // Saves this path to page "Path2"

            // building Path 3: Second custom curve from your visualizer (Uses 1 control point)
            Path3 = follower.pathBuilder() // Initiates building path segment 3
                    .addPath(
                            new BezierCurve( // Tells the computer this final track is also curved
                                    new Pose(30.000, 55.000),  // Must match ending point of Path 2
                                    new Pose(45.006, 94.996),  // Control Point : Bend
                                    new Pose(70.000, 70.000)   // Final field coordinate target
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90)) // heading change
                    .build(); // Saves this path to page "Path3"
        } // Closes the Paths constructor method
    } // Closes the static inner Paths class

    // --- THE CHECKLIST (FSM) ---
    public enum PathState {     // Defines the list of unique, customizable step states
        DRIVE_PATH1_LAUNCH,    // Step 1: Drive up the field via Path1
        LAUNCH_ELEMENT,         // Step 2: Stop and run physical launcher actions
        DRIVE_PATH_2,           // Step 3: Drive the first curve via Path2
        DRIVE_PATH_3,           // Step 4: Drive the second curve via Path3
        IDLE                    // Step 5: Safe parking mode; shuts down everything at end
    } // Closes the enumeration block

    // Sets program's starting state to point to the very first step on our checklist
    private PathState pathState = PathState.DRIVE_PATH1_LAUNCH;

    // Sets absolute starting location (X, Y, Heading) to align our localization system
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    // Helper method that instantiates the inner paths class and compiles the visualizer points
    private void buildPaths () {
        paths = new Paths(follower); // Feeds active follower into the blueprint to build the lines
    } // Closes path builder helper method

    // === 4. THE FINITE STATE MACHINE UPDATER ENGINE ===
    private void statePathUpdate(){ // Opens the method that evaluates your step execution every frame
        switch (pathState) {   // Evaluates which case block matches our current checklist step

            // ----------------------------------------------------
            // STATE: DRIVE PATH 1
            // ----------------------------------------------------
            case DRIVE_PATH1_LAUNCH:                               // If our state clipboard reads DRIVE_PATH1_LAUNCH:
                if (!follower.isBusy() && !pathStarted) {          // Gatekeeper: If robot isn't moving and path hasn't started yet:
                    follower.followPath(paths.Path1, 0.7, true);   // Send Path1 to motors, cap speed at 70%, hold point when done
                    pathTimer.resetTimer();                        // Send the step stopwatch back to 0.0 seconds
                    pathStarted = true;                            // Close the gatekeeper so we don't spam the motor commands
                }

                // 💡 ACTION SPOT (DURING PATH 1):
                // Because this code runs every single frame while driving Path 1, you can check pathTimer here!
                // Example: If you wanted to spin up the launcher halfway through driving:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    launcher.setTargetRpm(1100);
                    launcher.setAnglePosition(0.14);
                }

                // Transition Check: If robot is finished moving AND path started AND safety delay is satisfied
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(PathState.LAUNCH_ELEMENT); // Flip the clipboard step to launch the element
                }
                break; // Exit this switch block case safely

            // ----------------------------------------------------
            // STATE: LAUNCH ELEMENT (STOPPED ACTION USING TEAM METHODS)
            // ----------------------------------------------------
            case LAUNCH_ELEMENT: // If our state clipboard reads LAUNCH_ELEMENT:
                launcher.update(); // Tells your team's launcher internal loop to process commands
                if (!flywheelStarted) { // Gatekeeper: Runs only on the exact frame this state initializes
                    launcher.setTargetRpm(2200); // Uses your team's RPM method targeting 2200
                    launcher.setAnglePosition(0.14); // Uses your team's servo position encoder method
                    launcher.requestShot(); // Fires your team's automatic launcher trigger sequence
                    flywheelStarted = true; // Close the interlock gate so we don't waste CPU spamming motor lines
                }

                // Checks your team's internal busy-flag AND waits until 2 seconds are complete
                if (!launcher.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.DRIVE_PATH_2); // Advance the clipboard step to execute the Path 2 curve
                }
                break; // Exit this switch block case safely

            // ----------------------------------------------------
            // STATE: DRIVE PATH 2 (CURVE 1)
            // ----------------------------------------------------
            case DRIVE_PATH_2: // If our state clipboard reads DRIVE_PATH_2:
                if (!follower.isBusy() && !pathStarted) { // Gatekeeper: If robot isn't moving and track hasn't started:
                    follower.followPath(paths.Path2, 0.8, true); // Fire Path2 curve, cap speed at 80%, hold position firmly at end
                    pathTimer.resetTimer(); // Snap our step stopwatch back to 0.0 seconds
                    pathStarted = true; // Close the gatekeeper to prevent loop restarts
                }

                // 💡 ACTION SPOT (DURING PATH 2):
                // Put code here that you want to happen *while* the robot is actively sweeping along Curve 1.

                // Transition Check: Drivetrain finished curve AND track ran AND safety delay is satisfied
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.DRIVE_PATH_3); // Advance the clipboard step to execute the final Path 3 curve
                }
                break; // Exit this switch block case safely

            // ----------------------------------------------------
            // STATE: DRIVE PATH 3 (CURVE 2) — INTAKES MID-PATH!
            // ----------------------------------------------------
            case DRIVE_PATH_3: // If our state clipboard reads DRIVE_PATH_3:
                if (!follower.isBusy() && !pathStarted) { // Gatekeeper: If robot isn't moving and track hasn't started:
                    follower.followPath(paths.Path3, 0.8, true); // Fire Path3 curve, cap speed at 80%, lock end coordinate position
                    pathTimer.resetTimer(); // Snap our step stopwatch back to 0.0 seconds
                    pathStarted = true; // Close the gatekeeper variable
                }

                // 🌟 💡 ACTION SPOT (DURING PATH 3): TURNING ON MOTORS DURING A PATH
                // We check if the stopwatch has ticked past 1.0 second, AND ensure we haven't already fired the command.
                if (pathTimer.getElapsedTimeSeconds() >= 1.0 && !intakeStarted) {
                    IntakeStates.runForward(999999); // Turn on your intake motors forward using your team's custom method
                    intakeStarted = true;             // Close our intake gatekeeper so the motor line isn't spammed every frame
                }

                // Transition Check: Drivetrain landed on final field coordinate safely
                if (!follower.isBusy() && pathStarted && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(PathState.IDLE); // Advance the clipboard to the completed IDLE shutdown state
                }
                break; // Exit this switch block case safely

            // ----------------------------------------------------
            // STATE: SAFE MATCH CONCLUSION
            // ----------------------------------------------------
            case IDLE: // If our state clipboard reads IDLE:
                IntakeStates.stop();      // Force stop the intake motors so they don't break or build up heat
                launcher.setTargetRpm(0); // Safely spins down your team's launcher via RPM setting
                break; // Exit this switch block case safely
        } // Closes the switch block evaluation
    } // Closes the statePathUpdate manager method

    // === 5. STATE TRANSITION HELPER METHOD ===
    private void setPathState(PathState newState) { // Receives the next step name to transition to
        pathState = newState; // Updates our main clipboard tracker variable with the new step instruction
        pathTimer.resetTimer(); // Snaps our step stopwatch cleanly back to zero seconds for the new action
        pathStarted = false; // Re-opens the path gatekeeper variable so the next path is allowed to fire
        flywheelStarted = false; // Re-opens the hardware interlock gatekeeper variable for the next mechanical event
        intakeStarted = false; // 🌟 RESET GATEKEEPER: Re-opens the intake gatekeeper so future paths can use mid-drive intake actions!
    } // Closes state changer helper method

    // === 6. FTC OPMODE INITIALIZATION PHASE ===
    @Override // Overrides the standard init framework code block with our custom setup sequence
    public void init() { // Opens initialization method (Runs once when driver hits INIT button)
        pathTimer = new Timer(); // Instantiates physical stopwatch object in memory for managing state durations
        opModeTimer = new Timer(); // Instantiates physical stopwatch object in memory to track overall runtime

        // Initialize the Follower using your team's custom Constants profile configuration
        follower = Constants.createFollower(hardwareMap);

        launcher = new LauncherStates(hardwareMap); // Hooks up your team's custom launcher configuration mapping
        IntakeStates.init(hardwareMap);             // 🌟 INITIALIZED: Prepares your team's static Intake configuration mapping

        buildPaths(); // Runs helper method to load, construct, and save all field coordinates into memory
        follower.setPose(startPose); // Syncs PedroPathing's virtual map coordinates with the physical robot field placement
    } // Closes the initialization code block

    // === 7. FTC OPMODE START PHASE ===
    @Override // Overrides standard start framework code block
    public void start() { // Runs once the exact frame the driver presses the play triangle
        opModeTimer.resetTimer(); // Snap overall match clock back to zero seconds for full match tracking accuracy
        pathTimer.resetTimer(); // Snap state tracking stopwatch clock back to zero seconds
        pathState = PathState.DRIVE_PATH1_LAUNCH; // Force checklist to ensure it kicks off at the first step
        pathStarted = false; // Confirm path execution interlock gate is open and ready to fire
        flywheelStarted = false; // Confirm mechanism firing interlock gate is open and ready to fire
        intakeStarted = false; // 🌟 RESET ON START: Ensure intake flag is clean at match start
    } // Closes the start code block

    // === 8. FTC OPMODE DRIVER RUNTIME FRAME REPEATER ===
    @Override // Tells the phone to run this block over and over during the match
    public void loop() { // Opens loop method (Repeats continuously, roughly 60+ times a second during match)
        follower.update();    // Drives the background motor power calculations
        launcher.update();    // Constantly updates launcher PID loops throughout the entire autonomous match
        IntakeStates.update(); // 🌟 UPDATED: Keeps your team's intake hardware update loops running every single frame
        statePathUpdate();    // Jumps down to read our checklist steps every single frame

        // --- PREPARING DIAGNOSTIC TELEMETRY TO SHOW UP LIVE ON DRIVER HANDSET PHONE SCREEN ---
        telemetry.addData("Current Step", pathState); // Displays what step name the robot is actively running
        telemetry.addData("Robot Pose", follower.getPose().toString()); // Displays the live calculated field coordinates (X, Y, Angle)
        telemetry.addData("Path Started", pathStarted); // Displays whether a driving path command is actively running
        telemetry.addData("Follower Busy", follower.isBusy()); // Displays whether the drivetrain wheels are still navigating a vector
        telemetry.addData("Launcher Busy", launcher.isBusy()); // Displays if launcher macros are currently spinning/firing
        telemetry.addData("Intake State", IntakeStates.intakeState); // 🌟 NEW TELEMETRY: Shows active state of intake motor system
        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds()); // Displays total elapsed running time
        telemetry.update(); // Flushes data buffer to immediately push text telemetry onto the physical handset screen
    } // Closes the continuous loop method block
} // Closes the outer container MyPerfectAuto class block