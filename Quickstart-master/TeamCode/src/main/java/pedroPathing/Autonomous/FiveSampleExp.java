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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Robot.Commands.IntakeGrabAuto;
import pedroPathing.Robot.Commands.PDFLCommand;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.PIDController;
import pedroPathing.Robot.Sensing.PDFL;
import pedroPathing.Robot.Sensing.SleepCode;
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

@Autonomous(name = "Five (5) sample Exp", group = "Auto")
public class FiveSampleExp extends OpMode {

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
    private double deadzone = 25.0;
    private double homedConstant = 20;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int countA = 0;
    private int countB = 0;
    private int countX = 0;
    private int countY = 0;
    private long time;
    private boolean isBlockHorizontal = false;
    private boolean voidFifthBlock = false;
    boolean prevA = false;
    boolean prevB = false;
    boolean prevY = false;
    boolean prevX = false;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(16.5, 125, Math.toRadians(315));
    private final Pose scorePose1 = new Pose(18, 124, Math.toRadians(315));

    private final Pose scorePose2 = new Pose(19.5,127,Math.toRadians(315));
    private final Pose scorePose3 = new Pose(17.5,127,Math.toRadians(315));


    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(33, 119.5, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(33, 128, Math.toRadians(0));

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
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose1.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose3.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose2), /* Control Point */ new Point(parkControlPose), new Point(parkControlPose2),new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading());

        scoreSub1 = new Path(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(scorePose2)));
        scoreSub1.setLinearHeadingInterpolation(parkPose.getHeading(),scorePose2.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()){
                    liftsPlusDeposit(1475,0.6,20,Constants.outtakeSampleDrop[0],Constants.outtakeSampleDrop[1],Constants.outtakeSampleDrop[2],Constants.outtakeSampleDrop[3]);
                    SleepCode(0.15);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    SleepCode(0.25);
                    liftsPlusDeposit((int)homedConstant,0.35,20,0.48,0.52,Constants.outtakeSampleReset[2],Constants.outtakeSampleReset[3]);


                    if(pathTimer.getElapsedTimeSeconds() > 1.5){
                        follower.followPath(grabPickup1,true);
                        setPathState(2);
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                }
                break;
            case 2:
                if(follower.isBusy()){
                    intake.setServoPosition(1, 0.8);
                    intake.setServoPosition(2, 0.2);
                    intake.setServoPosition(3,0.5);
                    intake.setServoPosition(4,0.5);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    intake.setServoPosition(1, 0.85);
                    intake.setServoPosition(2, 0.15);
                    SleepCode(0.1);
                    intake.setServoPosition(6, 0.42);
                    SleepCode(0.2);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.isBusy()){
                    intake.setServoPosition(1, 0.6);
                    intake.setServoPosition(2, 0.4);
                    intake.setServoPosition(3,0.15);
                    intake.setServoPosition(4,0.85);
                    intake.setServoPosition(6, 0.45);
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    intake.setServoPosition(1, 0.6);
                    intake.setServoPosition(2, 0.4);
                    SleepCode(0.15);
                    deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
                    SleepCode(0.15);
                    intake.setServoPosition(6,Constants.intakeActive[4]);
                    SleepCode(0.2);
                    liftsPlusDeposit(1525,0.6,20,Constants.outtakeSampleDrop[0],Constants.outtakeSampleDrop[1],Constants.outtakeSampleDrop[2],Constants.outtakeSampleDrop[3]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    SleepCode(0.25);
                    liftsPlusDeposit((int)homedConstant-10,0.35,20,0.48,0.52,Constants.outtakeSampleReset[2],Constants.outtakeSampleReset[3]);



                    /* Score Sample */
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(follower.isBusy()){
                    intake.setServoPosition(1, 0.8);
                    intake.setServoPosition(2, 0.2);
                    intake.setServoPosition(3, 0.5);
                    intake.setServoPosition(4, 0.5);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intake.setServoPosition(1, 0.85);
                    intake.setServoPosition(2, 0.15);
                    SleepCode(0.1);
                    intake.setServoPosition(6, 0.42);
                    SleepCode(0.2);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(follower.isBusy()){
                    intake.setServoPosition(1, 0.6);
                    intake.setServoPosition(2, 0.4);
                    intake.setServoPosition(3,0.15);
                    intake.setServoPosition(4,0.85);
                    intake.setServoPosition(6, 0.45);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    intake.setServoPosition(1, 0.6);
                    intake.setServoPosition(2, 0.4);
                    SleepCode(0.15);
                    deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
                    SleepCode(0.15);
                    intake.setServoPosition(6,Constants.intakeActive[4]);
                    SleepCode(0.2);
                    liftsPlusDeposit(1525,0.6,20,Constants.outtakeSampleDrop[0],Constants.outtakeSampleDrop[1],Constants.outtakeSampleDrop[2],Constants.outtakeSampleDrop[3]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    SleepCode(0.25);
                    liftsPlusDeposit((int)homedConstant-10,0.35,20,0.48,0.52,Constants.outtakeSampleReset[2],Constants.outtakeSampleReset[3]);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(follower.isBusy()){
                    intake.setServoPosition(1, 0.83);
                    intake.setServoPosition(2, 0.17);
                    intake.setServoPosition(3,0.18);
                    intake.setServoPosition(4,0.18);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intake.setServoPosition(1, 0.85);
                    intake.setServoPosition(2, 0.15);
                    SleepCode(0.1);
                    intake.setServoPosition(6,0.42);
                    SleepCode(0.2);
                    new IntakeGrabAuto(intake).schedule();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(follower.isBusy() && follower.getPose().getX() < 30){
                    intake.setServoPosition(1, 0.6);
                    intake.setServoPosition(2, 0.4);
                    intake.setServoPosition(3,0.15);
                    intake.setServoPosition(4,0.85);
                    intake.setServoPosition(6, 0.45);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
                    SleepCode(0.25);
                    intake.setServoPosition(6,Constants.intakeActive[4]);
                    SleepCode(0.2);
                    liftsPlusDeposit(1525,0.6,20,Constants.outtakeSampleDrop[0],Constants.outtakeSampleDrop[1],Constants.outtakeSampleDrop[2],Constants.outtakeSampleDrop[3]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    SleepCode(0.15);
                    liftsPlusDeposit((int)homedConstant,0.35,20,0.48,0.52,Constants.outtakeSampleReset[2],Constants.outtakeSampleReset[3]);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(follower.isBusy()){
                    intake.setServoPosition(1, 0.8);
                    intake.setServoPosition(2, 0.2);
                    if(countA == 1) {
                        intake.setServoPosition(3, 0.18);
                        intake.setServoPosition(4, 0.18);
                    } else{
                        intake.setServoPosition(3, 0.5);
                        intake.setServoPosition(4, 0.5);
                    }
                    intake.setServoPosition(6, Constants.intakeActive[4]);

                }                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    PIDIntake(500,0.4,20);
                    SleepCode(0.2);
                    intake.setServoPosition(1, 0.85);
                    intake.setServoPosition(2, 0.15);
                    if(countA == 1) {
                        intake.setServoPosition(3, 0.18);
                        intake.setServoPosition(4, 0.18);
                    } else{
                        intake.setServoPosition(3, 0.5);
                        intake.setServoPosition(4, 0.5);
                    }
                    intake.setServoPosition(6,0.42);
                    SleepCode(0.5);
                    PIDIntake(-30,0.4,20);

                    follower.followPath(scoreSub1);
                    setPathState(9);
                }
                break;
            case 9:

                if(!follower.isBusy()) {
                    /* Score Sample */
                    deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
                    SleepCode(0.15);
                    intake.setServoPosition(6,Constants.intakeActive[4]);
                    SleepCode(0.2);
                    liftsPlusDeposit(1525,0.6,20,Constants.outtakeSampleDrop[0],Constants.outtakeSampleDrop[1],Constants.outtakeSampleDrop[2],Constants.outtakeSampleDrop[3]);
                    deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
                    SleepCode(0.25);
                    liftsPlusDeposit(0,0.35,20,0.48,0.52,Constants.outtakeSampleReset[2],Constants.outtakeSampleReset[3]);
                    deposit.setServoPosition(1, Constants.outtakeSampleDrop[0]);
                    deposit.setServoPosition(2, Constants.outtakeSampleDrop[1]);
                    deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);
                    deposit.setServoPosition(4,Constants.outtakeSampleDrop[3]);
                    setPathState(-1);
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        lifts.periodic();

        if(buzzerOveride()){
            setPathState(-1);
            stop();
        }

        if(intake.getExtensionPosition() >= 0){
            intake.setMotorPower(-0.1);
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        AutonTimer = new Timer();
        opmodeTimer.resetTimer();
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        lifts = new Lifts(hardwareMap, voltageSensor.getVoltage());
        follower.setStartingPose(startPose);

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


        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Right Bumper = Horizonral",0);
        telemetry.addData("Left Bumper = void 5th Block",1);
        telemetry.addData("A -> +1 x && B -> -1 x",2);
        telemetry.addData("Y -> +1 y && X -> -1 y",3);

        // Set Block Horizontal (only once)
        if (gamepad1.right_bumper) {
            isBlockHorizontal = true;
            if (countA == 0) {  // Prevents multiple increments
                countA = 1;
            }
            telemetry.addData("Horizontal Spin:", countA);
        }

        // Set Void Fifth Block (only once)
        if (gamepad1.left_bumper) {
            voidFifthBlock = true;
            if (countB == 0) {  // Prevents multiple increments
                countB = 1;
            }
            telemetry.addData("Void 5th Block", countB);
        }

        // Detect "A" button press to increase X **only once per press**
        if (gamepad1.a && !prevA) {
            countX += 1;
        }
        prevA = gamepad1.a; // Save button state

        // Detect "B" button press to decrease X **only once per press**
        if (gamepad1.b && !prevB) {
            countX -= 1;
        }
        prevB = gamepad1.b;

        // Detect "Y" button press to increase Y **only once per press**
        if (gamepad1.y && !prevY) {
            countY += 1;
        }
        prevY = gamepad1.y;

        // Detect "X" button press to decrease Y **only once per press**
        if (gamepad1.x && !prevX) {
            countY -= 1;
        }
        prevX = gamepad1.x;


        // Update telemetry with correct values
        telemetry.addData("Fifth X Addition", countX);
        telemetry.addData("Fifth Y Addition", countY);
        telemetry.update();
        buildPaths();
        intake.setMotor(0);
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        AutonTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void SleepCode(double seconds){
        actionTimer.resetTimer();
        while(actionTimer.getElapsedTimeSeconds() < seconds){
            telemetry.addLine("Sleep Code is Running");
            telemetry.update();
        }
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

    public void loopLifts(int target, double totalTimeinSeconds, int numberOfLoops){
        for (int i = 0; i < numberOfLoops; i++) {
            PDFL(target);
            SleepCode(totalTimeinSeconds / numberOfLoops);
        }
        lifts.SetPower(kF);
    }

    public void liftsPlusDeposit(int target, double totalTimeinSeconds, int numberOfLoops, double position1, double position2, double position3, double position4){
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
            SleepCode(totalTimeinSeconds / numberOfLoops);
        }
        lifts.SetPower(kF);
    }

    public void PIDIntake(int target, double totalTimeinSeconds, int numberOfLoops){
        for (int i = 0; i < numberOfLoops; i++) {
            if(target < 20){
                intake.setServoPosition(1, 0.6);
                intake.setServoPosition(2, 0.4);
                intake.setServoPosition(3,0.15);
                intake.setServoPosition(4,0.85);
                intake.setServoPosition(6, 0.45);
            }
            double power = pidController.compute(intake.getExtensionPosition(),target);
            intake.setMotorPower(power);
            SleepCode(totalTimeinSeconds / numberOfLoops);
        }
        intake.setMotorPower(0.075);


    }

    public boolean buzzerOveride(){
        if(AutonTimer.getElapsedTimeSeconds() > 32){
            return true;
        }
        return false;
    }



}

