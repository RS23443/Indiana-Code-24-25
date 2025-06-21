package pedroPathing.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.PIDController;
import pedroPathing.Robot.Sensing.PDFL;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.Robot.Systems.*;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Parellel Exp", group = "Auto")
public class ParellelExp extends OpMode {

    private Follower follower;
    private Intake intake;
    private Deposit deposit;
    private Lifts lifts;
    private VoltageSensor voltageSensor;

    private Timer pathTimer, actionTimer, opmodeTimer, AutonTimer;

    private PDFL pdfLController;
    private PIDController pidController;
    public double kPi = 0.01, kIi = 0.00, kDi = 0.000, alphai = 0.075;
    private double kP = 0.008, kD = 0.003, kF = 0.133, kL = 0.1;
    private double deadzone = 25.0; // Deadzone for lift/intake target checks
    private double homedConstant = 0;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int countA = 0;
    private int countB = 0;
    private int countX = 0;
    private int countY = 0;
    private boolean isBlockHorizontal = false;
    private boolean voidFifthBlock = false;
    boolean prevA = false;
    boolean prevB = false;
    boolean prevY = false;
    boolean prevX = false;

    // State machine variables for actions (Intake, Deposit, Lifts)
    private TrasferState currentActionState;
    public enum TrasferState{
        INTAKEDROPDOWN,
        INTAKEGRAB,
        INTAKEFLIPIN,
        INTAKEOPEN,
        DEPOSITCLOSE,
        DEPOSITOPEN,
        DEPOSITFLIP,
        DEPOSITRESET,
        LIFTUP,
        LIFTDOWN
    }

    private boolean isActionSequenceActive = false;    // True if an action sequence is currently running
    private boolean isActionSequenceComplete = false;  // True when the current action sequence has finished

    // Constants for timing servo movements (these need tuning on the robot)
    // These values represent the estimated time it takes for a servo to reach its position.
    private static final double SERVO_DROP_TIME = 0.5; // seconds
    private static final double SERVO_GRAB_TIME = 0.5; // seconds
    private static final double SERVO_FLIP_TIME = 0.5; // seconds
    private static final double DEPOSIT_CLOSE_TIME = 0.5; // seconds
    private static final double INTAKE_OPEN_TIME = 0.5; // seconds
    private static final double DEPOSIT_FLIP_TIME = 0.5; // seconds
    private static final double DEPOSIT_OPEN_TIME = 0.5; // seconds
    private static final double DEPOSIT_RESET_TIME = 0.5; // seconds

    public double powerIntake, powerLifts;
    public int tragetIntake, targetLifts;


    /* Create and Define Poses + Paths */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(15.5, 126, Math.toRadians(315));
    private final Pose scorePose2 = new Pose(19.5,127,Math.toRadians(315));


    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(34, 119.5, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(34, 128, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(43, 128.5, Math.toRadians(90));

    /** Park Pose for our robot, after we do all of the scoring. */
    private Pose parkPose;

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(30, 128, Math.toRadians(90));
    private final Pose parkControlPose2 = new Pose(50,120,Math.toRadians(180));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park, scoreSub1;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        parkPose = new Pose(63 + countX, 98 + countY, Math.toRadians(270));

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose2), /* Control Point */ new Point(parkControlPose), new Point(parkControlPose2),new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading());

        scoreSub1 = new Path(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(scorePose2)));
        scoreSub1.setLinearHeadingInterpolation(parkPose.getHeading(),scorePose2.getHeading());
    }

    /** This method manages the progression of paths and actions.
     * It uses a state machine (`pathState`) for overall autonomous flow.
     * It also uses flags (`isActionSequenceActive`, `isActionSequenceComplete`)
     * to manage non-blocking action sequences (intake, deposit, lifts) handled by `checkState()`.
     */
    public void autonomousPathUpdate() {
        // If an action sequence is active, prioritize its execution.
        // `checkState()` will run one step per loop iteration.
        // Pathing logic is suspended until the action sequence is complete.
        if (isActionSequenceActive) {
            checkState(); // Run the current step of the action sequence
            if (isActionSequenceComplete) {
                isActionSequenceActive = false;    // Reset active flag
                isActionSequenceComplete = false;  // Reset complete flag
                // pathState will automatically advance in the next loop iteration
                // because the condition to wait for action completion will now be met.
            }
            return; // Exit this method to ensure only action logic runs if an action is active
        }

        // Main autonomous path progression logic
        switch (pathState) {
            case 0: // Start by scoring preload
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1: // Wait for scorePreload to finish, then trigger initial action sequence
                if(follower.isBusy()){
                    liftsPlusDeposit(1525,50,Constants.outtakeSampleDrop[0],Constants.outtakeSampleDrop[1],Constants.outtakeSampleDrop[2],Constants.outtakeSampleDrop[3]);
                }

                if (!follower.isBusy()) {
                    // Initialize the action sequence (e.g., deposit open after scoring preload)
                    isActionSequenceActive = true;
                    isActionSequenceComplete = false;
                    //currentActionState = TrasferState.DEPOSITOPEN; // Start with opening deposit
                    actionTimer.resetTimer(); // Reset timer for the first action state
                    setPathState(-1); // Move to the state that waits for the action
                }
                break;
            case 2: // Wait for the action sequence to complete after scorePreload
                if (!isActionSequenceActive && isActionSequenceComplete) {
                    // Action sequence finished, now proceed with the next path
                   // follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3: // Wait for grabPickup1 to finish, then trigger action (grab the sample)
                if (!follower.isBusy()) {
                    isActionSequenceActive = true;
                    isActionSequenceComplete = false;
                    currentActionState = TrasferState.INTAKEDROPDOWN; // Start the intake-grab sequence
                    actionTimer.resetTimer();
                    setPathState(4); // Move to the state that waits for the action
                }
                break;
            case 4: // Wait for the action sequence to complete (grab sample)
                if (!isActionSequenceActive && isActionSequenceComplete) {
                    // Action sequence finished, now proceed to score the sample
                  //  follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5: // Wait for scorePickup1 to finish, then trigger action (release and reset for next grab)
                if (!follower.isBusy()) {
                    isActionSequenceActive = true;
                    isActionSequenceComplete = false;
                    currentActionState = TrasferState.DEPOSITRESET; // Assuming this resets deposit after scoring
                    actionTimer.resetTimer();
                    setPathState(6); // Move to the state that waits for the action
                }
                break;
            case 6: // Wait for action completion, then grab next sample
                if (!isActionSequenceActive && isActionSequenceComplete) {
                    //follower.followPath(grabPickup2, true);
                    setPathState(7);
                }
                break;
            case 7: // Wait for grabPickup2 to finish, then trigger action (score the second sample)
                if (!follower.isBusy()) {
                    isActionSequenceActive = true;
                    isActionSequenceComplete = false;
                    currentActionState = TrasferState.DEPOSITFLIP; // Start scoring sequence
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8: // Wait for action completion, then score next sample
                if (!isActionSequenceActive && isActionSequenceComplete) {
                    //follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;
            case 9: // Wait for scorePickup2 to finish, then trigger action (release and reset for next grab)
                if (!follower.isBusy()) {
                    isActionSequenceActive = true;
                    isActionSequenceComplete = false;
                    currentActionState = TrasferState.DEPOSITRESET; // Reset deposit
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10: // Wait for action completion, then grab third sample
                if (!isActionSequenceActive && isActionSequenceComplete) {
                    //follower.followPath(grabPickup3, true);
                    setPathState(11);
                }
                break;
            case 11: // Wait for grabPickup3 to finish, then trigger action (score the third sample)
                if (!follower.isBusy()) {
                    isActionSequenceActive = true;
                    isActionSequenceComplete = false;
                    currentActionState = TrasferState.DEPOSITFLIP; // Start scoring sequence
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12: // Wait for action completion, then score third sample
                if (!isActionSequenceActive && isActionSequenceComplete) {
                    //follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;
            case 13: // Wait for scorePickup3 to finish, then park
                if (!follower.isBusy()) {
                    //follower.followPath(park, true);
                    setPathState(14);
                }
                break;
            case 14: // Wait for park to finish
                if (!follower.isBusy()){
                    // All autonomous actions are done.
                    // If scoreSub1 is meant to be after park, you could put it here:
                    // follower.followPath(scoreSub1, false);
                    setPathState(-1); // End the auto
                }
                break;
            case -1: // Autonomous sequence finished
                // The robot is idle.
                break;
        }
    }

    /** Changes the overall path state and resets the path timer. **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        CommandScheduler.getInstance().run(); // Manages FTCLib commands, if any are used separately
        lifts.periodic(); // Update lift system (e.g., motor encoder readings, PID calculations)

        // Continuously calculate and apply power to intake and lifts based on their current targets.
        // This ensures non-blocking control for these systems.
        powerIntake = powerForIntakeMotion(tragetIntake);
        intake.setMotorPower(powerIntake);


        // Emergency stop if autonomous time limit is exceeded
        if(buzzerOveride()){
            setPathState(-1); // Move to end state for pathing
            stop(); // Immediately stop the OpMode (and thus all motors)
            return; // Exit loop to prevent any further execution
        }

        // This line might interfere with PID control if the intake is supposed to be holding
        // a different position or moving. Review its necessity.
        // If the intake PID is handling holding at 0, this can be removed.
        if(intake.getExtensionPosition() >= 0){
            intake.setMotorPower(-0.1); // Small power to retract/hold?
        }

        // Update odometry and follower, and manage the autonomous state machines
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging and monitoring
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", currentActionState);
        telemetry.addData("Action Sequence Active", isActionSequenceActive);
        telemetry.addData("Action Sequence Complete", isActionSequenceComplete);
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", follower.getPose().getHeading());
        telemetry.addData("Lift Position", lifts.getTopMotorData()[0]);
        telemetry.addData("Intake Extension Position", intake.getExtensionPosition());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        CommandScheduler.getInstance().reset(); // Reset FTCLib command scheduler
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        AutonTimer = new Timer();
        opmodeTimer.resetTimer();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        lifts = new Lifts(hardwareMap, voltageSensor.getVoltage());
        follower.setStartingPose(startPose);

        // Set initial servo positions
        intake.setServoPosition(1, 0.8);
        intake.setServoPosition(2, 0.2);
        intake.setServoPosition(3,0.5);
        intake.setServoPosition(4,0.5);
        intake.setServoPosition(6, 0.7);

        deposit.setServoPosition(1,0.26);
        deposit.setServoPosition(2,0.74);
        deposit.setServoPosition(3,0.35);
        deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);

        pdfLController = new PDFL(kP, kD, kF, kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);
        pidController = new PIDController(kPi,kIi,kDi,alphai);

        powerIntake = 0.0; // Initialize power to 0
        powerLifts = 0.0;  // Initialize power to 0
        targetLifts = 0;   // Initialize target for lifts to 0 (home position)
        tragetIntake = 0;  // Initialize target for intake to 0 (home position)

        currentActionState = TrasferState.INTAKEDROPDOWN; // Initialize action state machine to its first step

        buildPaths(); // Build all path objects
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        // Telemetry for driver control during init_loop
        telemetry.addData("Right Bumper = Horizonral",0);
        telemetry.addData("Left Bumper = void 5th Block",1);
        telemetry.addData("A -> +1 x && B -> -1 x",2);
        telemetry.addData("Y -> +1 y && X -> -1 y",3);

        // Gamepad input for adjusting parking pose or other init parameters
        if (gamepad1.right_bumper) {
            isBlockHorizontal = true;
            if (countA == 0) {
                countA = 1;
            }
            telemetry.addData("Horizontal Spin:", countA);
        }

        if (gamepad1.left_bumper) {
            voidFifthBlock = true;
            if (countB == 0) {
                countB = 1;
            }
            telemetry.addData("Void 5th Block", countB);
        }

        if (gamepad1.a && !prevA) {
            countX += 1;
        }
        prevA = gamepad1.a;

        if (gamepad1.b && !prevB) {
            countX -= 1;
        }
        prevB = gamepad1.b;

        if (gamepad1.y && !prevY) {
            countY += 1;
        }
        prevY = gamepad1.y;

        if (gamepad1.x && !prevX) {
            countY -= 1;
        }
        prevX = gamepad1.x;

        telemetry.addData("Fifth X Addition", countX);
        telemetry.addData("Fifth Y Addition", countY);
        telemetry.update();

        buildPaths(); // Rebuild paths in case countX/countY changed
        intake.setMotor(0); // Ensure intake motor is off during init_loop
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        AutonTimer.resetTimer();
        setPathState(0); // Start the main autonomous sequence from state 0
        // Ensure action sequence flags are reset at the start of auto
        isActionSequenceActive = false;
        isActionSequenceComplete = false;
        currentActionState = TrasferState.INTAKEDROPDOWN; // Set action state machine to its first step
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        // Ensure all robot movement systems are explicitly stopped
        intake.setMotorPower(0);
        lifts.SetPower(0);
    }

    /** Checks if the autonomous time limit has been exceeded. **/
    public boolean buzzerOveride(){
        if(AutonTimer.getElapsedTimeSeconds() > 32){ // Typically 30 seconds for auto period
            return true;
        }
        return false;
    }

    /** Calculates the motor power for lift movement using PDFL control. **/
    public double PDFLPower(int target){
        pdfLController.updateConstants(kP, kD, kF, kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);
        int pos = (int) lifts.getTopMotorData()[0];
        double error2 = target - pos;
        double powerTop = pdfLController.run(error2);
        powerTop = Math.max(-1, Math.min(powerTop, 1));
        return powerTop;
    }

    /** This method manages the state transitions for intake, deposit, and lift actions.
     * It relies on timers for servo movements and position checks for motor movements
     * to ensure non-blocking execution and proper sequencing of actions.
     */
    public void checkState(){
        switch (currentActionState){
            case INTAKEDROPDOWN:
                intake.setServoPosition(1, 0.83);
                intake.setServoPosition(2, 0.18);
                intake.setServoPosition(3,0.5);
                intake.setServoPosition(4,0.5);
                intake.setServoPosition(6, Constants.intakeActive[4]);
                if (actionTimer.getElapsedTimeSeconds() > SERVO_DROP_TIME) {
                    actionTimer.resetTimer(); // Reset timer for the next state
                    currentActionState = TrasferState.INTAKEGRAB;
                }
                break;
            case INTAKEGRAB:
                intake.setServoPosition(1, 0.85);
                intake.setServoPosition(2, 0.15);
                intake.setServoPosition(6, 0.42);
                if (actionTimer.getElapsedTimeSeconds() > SERVO_GRAB_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.INTAKEFLIPIN;
                }
                break;
            case INTAKEFLIPIN:
                intake.setServoPosition(1, 0.6);
                intake.setServoPosition(2, 0.4);
                intake.setServoPosition(3,0.15);
                intake.setServoPosition(4,0.85);
                intake.setServoPosition(6, 0.45);
                if (actionTimer.getElapsedTimeSeconds() > SERVO_FLIP_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.DEPOSITCLOSE;
                }
                break;
            case DEPOSITCLOSE:
                deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
                if (actionTimer.getElapsedTimeSeconds() > DEPOSIT_CLOSE_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.INTAKEOPEN;
                }
                break;
            case INTAKEOPEN:
                intake.setServoPosition(6,Constants.intakeActive[4]);
                if (actionTimer.getElapsedTimeSeconds() > INTAKE_OPEN_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.DEPOSITFLIP;
                }
                break;
            case LIFTUP:
                targetLifts = 1500; // Set target for lifts. Lifts will move towards this target continuously in loop().
                // Wait until lifts are close enough to the target
                if (Math.abs(lifts.getTopMotorData()[0] - targetLifts) < deadzone) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.DEPOSITFLIP;
                }
                break;
            case DEPOSITFLIP:
                deposit.setServoPosition(1, Constants.outtakeSampleDrop[0]);
                deposit.setServoPosition(2, Constants.outtakeSampleDrop[1]);
                deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);
                if (actionTimer.getElapsedTimeSeconds() > DEPOSIT_FLIP_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.DEPOSITOPEN;
                }
                break;
            case DEPOSITOPEN:
                deposit.setServoPosition(4, Constants.outtakeSampleReset[3]);
                if (actionTimer.getElapsedTimeSeconds() > DEPOSIT_OPEN_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.DEPOSITRESET;
                }
                break;
            case DEPOSITRESET:
                deposit.setServoPosition(1, Constants.outtakeSampleReset[0]);
                deposit.setServoPosition(2, Constants.outtakeSampleReset[1]);
                deposit.setServoPosition(3, Constants.outtakeSampleReset[2]);
                if (actionTimer.getElapsedTimeSeconds() > DEPOSIT_RESET_TIME) {
                    actionTimer.resetTimer();
                    currentActionState = TrasferState.INTAKEDROPDOWN;
                }
                break;
            case LIFTDOWN:
                targetLifts = 50; // Set target for lifts (down position).
                // Wait until lifts are close enough to the target
                if (Math.abs(lifts.getTopMotorData()[0] - targetLifts) < deadzone) {
                    // The entire action sequence is now complete
                    isActionSequenceComplete = true; // Signal completion to autonomousPathUpdate
                    // Reset currentActionState to the beginning for next time it's triggered
                    currentActionState = TrasferState.INTAKEDROPDOWN;
                }
                break;
        }
    }

    /** Calculates the motor power for intake extension using PID control. **/
    public double powerForIntakeMotion(int target){
        powerIntake = pidController.compute(intake.getExtensionPosition(),target);
        return powerIntake;
    }

    public double powerForLiftMotion(int target){
        powerLifts = PDFLPower(target);
        return powerLifts;
    }
    public void PDFL(int target){
        int pos = (int)lifts.getTopMotorData()[0];
        int direction = target-pos;
        if(direction < 0 && pos >= target + deadzone) {
            pdfLController.updateConstants(kP, kD, kF, kL);
            pdfLController.setDeadzone(deadzone);
            pdfLController.setHomedConstant(homedConstant);
            pos = (int) lifts.getTopMotorData()[0];
            double error2 = target - pos;
            double powerTop = pdfLController.run(error2);
            powerTop = Math.max(-1, Math.min(powerTop, 1));
            lifts.SetPower(powerTop);
        } else if (direction > 0 && pos <= target -deadzone){
            pdfLController.updateConstants(kP, kD, kF, kL);
            pdfLController.setDeadzone(deadzone);
            pdfLController.setHomedConstant(homedConstant);
            pos = (int) lifts.getTopMotorData()[0];
            double error2 = target - pos;
            double powerTop = pdfLController.run(error2);
            powerTop = Math.max(-1, Math.min(powerTop, 1));
            lifts.SetPower(powerTop);
        } else {
            lifts.SetPower(kF/voltageSensor.getVoltage());
        }
    }

    public void liftsPlusDeposit(int target, int numberOfLoops, double position1, double position2, double position3, double position4){
        for (int i = 0; i < numberOfLoops; i++) {
            if(target >1000) {
                if (lifts.getTopMotorData()[0] > 3*target /5 ) {
                    deposit.setServoPosition(1, position1);
                    deposit.setServoPosition(2, position2);
                    deposit.setServoPosition(3, position3);
                    deposit.setServoPosition(4, position4);
                }
            } else{
                deposit.setServoPosition(1, position1);
                deposit.setServoPosition(2, position2);
                deposit.setServoPosition(3, position3);
                deposit.setServoPosition(4, position4);

            }

            PDFL(target);

        }
        lifts.SetPower(kF);
    }

}
