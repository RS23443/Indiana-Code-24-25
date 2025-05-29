package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines.ContrastAndROI;
import org.firstinspires.ftc.teamcode.Robot.Constants; // Your Constants file
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;   // Your Intake subsystem
import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;    // Your Lifts subsystem
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;  // Your Deposit subsystem


import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RoadRunner Full Auto (Subsystems V3)", group = "Autonomous")
// @Disabled
public class RoadRunnerVisionAutonomous extends LinearOpMode {

    MecanumDrive drive;
    Pose2d currentPose;
    public VoltageSensor controlHubVoltageSensor;

    OpenCvCamera webcam;
    ContrastAndROI visionPipeline;
    DetectionResult detectedPosition = DetectionResult.NONE;
    int cameraWidth = 640; int cameraHeight = 480;
    Rect inspectionROI;
    private volatile boolean isWebcamStreaming = false;
    private static final double SCAN_DURATION_SEC = 1.5;
    private ElapsedTime scanTimer = new ElapsedTime();
    private static final int MAX_SCAN_ATTEMPTS = 2;
    private static final double INCREMENTAL_MOVE_DISTANCE = 6.0;

    public Intake intake;
    public Lifts lifts;
    public Deposit deposit;

    public enum DetectionResult { LEFT, CENTER, RIGHT, NONE }

    // --- Action Classes ---

    public class SetIntakeServosAction implements Action {
        private final double[] targetPositions;
        private boolean initialized = false;
        private static final double SERVO_ACTION_DURATION = 0.4; // Faster assumption
        private ElapsedTime timer = new ElapsedTime();
        public SetIntakeServosAction(double[] positions) { this.targetPositions = positions; }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // [0]:lif, [1]:rif, [2]:leftdiffy, [3]:rightdiffy, [4]:intake_claw
                intake.setServoPosition(1, targetPositions[0]);
                intake.setServoPosition(2, targetPositions[1]);
                intake.setServoPosition(3, targetPositions[2]);
                intake.setServoPosition(4, targetPositions[3]);
                intake.setServoPosition(5, targetPositions[4]);
                timer.reset(); initialized = true;
                packet.put("IntakeServosAction", "Setting to " + targetPositions[4]);
            }
            if (timer.seconds() > SERVO_ACTION_DURATION) return false;
            return true;
        }
    }

    public class MoveIntakeExtensionPIDAction implements Action {
        private final int targetTicks;
        private boolean initialized = false;
        private ElapsedTime timeoutTimer = new ElapsedTime();
        private static final double EXTENSION_TIMEOUT_SEC = 2.5;
        private static final int POSITION_TOLERANCE = 20; // Ticks

        public MoveIntakeExtensionPIDAction(int targetTicks) { this.targetTicks = targetTicks; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!opModeIsActive()) { intake.setMotorPower(0); return false;}

            if (!initialized) {
                telemetry.addData("IntakeExtPIDAction: Moving to", targetTicks);
                // PID constants should be set in Intake.java constructor
                // Ensure Intake motor is in RUN_WITHOUT_ENCODER mode
                initialized = true;
                timeoutTimer.reset();
            }

            // The setMotor method in your Intake class already does PID compute and setPower
            intake.setMotor(targetTicks); // This line calls your PID logic

            int currentPos = intake.getExtensionPosition();
            boolean atTarget = Math.abs(currentPos - targetTicks) < POSITION_TOLERANCE;

            packet.put("IntakeExt Target", targetTicks);
            packet.put("IntakeExt CurrentPos", currentPos);
            packet.put("IntakeExt AtTarget", atTarget);

            if (atTarget) {
                telemetry.addLine("IntakeExtPIDAction: Reached target.");
                intake.setMotorPower(0); // Hold position or allow PID to do so if kI is well-tuned
                return false;
            }
            if (timeoutTimer.seconds() > EXTENSION_TIMEOUT_SEC) {
                telemetry.addLine("IntakeExtPIDAction: Timed out.");
                intake.setMotorPower(0);
                return false;
            }
            return true;
        }
    }


    public class MoveLiftPDFLAction implements Action {
        private final int targetTicks;
        private boolean initialized = false;
        private ElapsedTime timeoutTimer = new ElapsedTime();
        private static final double LIFT_TIMEOUT_SEC = 3.5;
        private static final int POSITION_TOLERANCE = 25;

        public MoveLiftPDFLAction(int targetTicks) { this.targetTicks = targetTicks; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!opModeIsActive()) { lifts.enablePDControl(false); lifts.stopSlides(); return false; }
            if (!initialized) {
                lifts.setTarget(targetTicks);
                lifts.enablePDControl(true);
                initialized = true; timeoutTimer.reset();
                packet.put("LiftPDFLAction", "Init to " + targetTicks);
            }
            lifts.updatePDControl();
            double currentTopPos = lifts.getTopMotorData()[0];
            double middleMotorVel = lifts.getMiddleMotorData()[1];
            double bottomMotorVel = lifts.getBottomMotorData()[1];
            double topMotorVel = lifts.getTopMotorData()[1];
            boolean positionReached = Math.abs(currentTopPos - targetTicks) < POSITION_TOLERANCE;
            boolean anyMotorEffectivelyStopped = Math.abs(middleMotorVel) < 10 && Math.abs(bottomMotorVel) < 10 && Math.abs(topMotorVel) < 10; // Adjusted velocity check

            packet.put("LiftPDFL Target", targetTicks);
            packet.put("LiftPDFL TopPos", String.format("%.1f", currentTopPos));
            packet.put("LiftPDFL PosReached", positionReached);
            packet.put("LiftPDFL MotorsStopped", anyMotorEffectivelyStopped);

            if (positionReached && anyMotorEffectivelyStopped) {
                lifts.runslides(lifts.DynamicKF()); lifts.enablePDControl(false);
                return false;
            }
            if (timeoutTimer.seconds() > LIFT_TIMEOUT_SEC) {
                lifts.runslides(lifts.DynamicKF()); lifts.enablePDControl(false);
                return false;
            }
            return true;
        }
    }

    public class SetDepositServosAction implements Action {
        private final double[] targetPositions;
        private boolean initialized = false;
        private static final double SERVO_ACTION_DURATION = 0.6; // Faster
        private ElapsedTime timer = new ElapsedTime();
        public SetDepositServosAction(double[] positions) { this.targetPositions = positions; }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // [0]:lof, [1]:rof, [2]:right_elbow, [3]:o_claw
                deposit.setServoPosition(1, targetPositions[0]);
                deposit.setServoPosition(2, targetPositions[1]);
                deposit.setServoPosition(3, targetPositions[2]);
                deposit.setServoPosition(4, targetPositions[3]);
                timer.reset(); initialized = true;
                packet.put("DepositServosAction", "Setting to " + targetPositions[3]);
            }
            if (timer.seconds() > SERVO_ACTION_DURATION) return false;
            return true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        currentPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, currentPose);

        double currentVoltage = 12.0;
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        try { currentVoltage = controlHubVoltageSensor.getVoltage(); }
        catch (Exception e) { telemetry.addLine("No voltage sensor? Using 12V default for Lifts."); }

        lifts = new Lifts(hardwareMap, currentVoltage);
        intake = new Intake(hardwareMap); // Make sure Intake's PID constants (kP,kI,kD) are set!
        deposit = new Deposit(hardwareMap);

        int roiWidth = 200; int roiHeight = 200;
        int roiX = (cameraWidth - roiWidth) / 2; int roiY = (cameraHeight - roiHeight) / 2;
        inspectionROI = new Rect(roiX, roiY, roiWidth, roiHeight);
        initializeVision();

        telemetry.addData("Status", "Initialized. Voltage: " + String.format("%.2f", currentVoltage) + ". Waiting for Start...");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", isWebcamStreaming ? "Webcam Streaming. Ready." : "Waiting for webcam...");
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) { cleanup(); return; }
        if (!isWebcamStreaming) {
            telemetry.addLine("Webcam did not start. Aborting.");
            telemetry.update(); sleep(2000); cleanup(); return;
        }

        List<Action> actionsInProgress = new ArrayList<>();
        boolean yellowFoundAndActed = false;
        TrajectoryActionBuilder tBuilder;

        tBuilder = drive.actionBuilder(currentPose);
        tBuilder.lineToYSplineHeading(15, Math.toRadians(0)); // EXAMPLE
        actionsInProgress.add(tBuilder.build());
        currentPose = tBuilder.endPose;
        actionsInProgress.add(new Action() { @Override public boolean run(@NonNull TelemetryPacket p) { performScan(); return false; } });
        Actions.runBlocking(new SequentialAction(actionsInProgress));
        actionsInProgress.clear();

        if (detectedPosition != DetectionResult.NONE) {
            telemetry.addLine("Yellow found on initial scan!");
            addFullGrabAndScoreSequence(actionsInProgress, currentPose);
            yellowFoundAndActed = true;
        } else {
            telemetry.addLine("No yellow on initial scan. Starting incremental scan...");
            for (int attempt = 0; attempt < MAX_SCAN_ATTEMPTS && !yellowFoundAndActed && opModeIsActive(); attempt++) {
                if (isStopRequested()) break;
                telemetry.addData("Incremental Scan Attempt", (attempt + 1));
                tBuilder = drive.actionBuilder(currentPose);
                tBuilder.forward(INCREMENTAL_MOVE_DISTANCE);
                actionsInProgress.add(tBuilder.build());
                currentPose = tBuilder.endPose;
                actionsInProgress.add(new Action() { @Override public boolean run(@NonNull TelemetryPacket p) { performScan(); return false; } });
                Actions.runBlocking(new SequentialAction(actionsInProgress));
                actionsInProgress.clear();

                if (detectedPosition != DetectionResult.NONE) {
                    telemetry.addLine("Yellow found on incremental scan!");
                    addFullGrabAndScoreSequence(actionsInProgress, currentPose);
                    yellowFoundAndActed = true;
                    break;
                }
            }
        }

        tBuilder = drive.actionBuilder(currentPose);
        if (!yellowFoundAndActed) {
            telemetry.addLine("No yellow found. Parking.");
            tBuilder.lineToLinearHeading(new Pose2d(currentPose.position.x + 10, currentPose.position.y - 10, Math.toRadians(0))); // EXAMPLE
        } else {
            telemetry.addLine("Actions complete. Parking.");
            tBuilder.lineToLinearHeading(new Pose2d(currentPose.position.x + 5, currentPose.position.y + 5, Math.toRadians(90))); // EXAMPLE
        }
        actionsInProgress.add(tBuilder.build());
        currentPose = tBuilder.endPose; // Update currentPose to the final parking spot start

        if (!actionsInProgress.isEmpty()) {
            telemetry.addLine("Executing final sequence...");
            Actions.runBlocking(new SequentialAction(actionsInProgress));
        }

        telemetry.addData("Status", "Autonomous Finished");
        telemetry.update();
        sleep(1000);
        cleanup();
    }

    private void addFullGrabAndScoreSequence(List<Action> sequence, Pose2d startOfSequencePose) {
        // This method will update the OpMode's main `currentPose` after drive actions
        this.currentPose = startOfSequencePose; // Start with the passed pose

        // 1. Prepare Intake & Extend
        // IMPORTANT: Define Constants.INTAKE_EXTEND_TARGET_TICKS in your Constants.java
        sequence.add(new SetIntakeServosAction(Constants.intakeActive)); // Open claw, set flips/diffy
        sequence.add(new MoveIntakeExtensionPIDAction(Constants.intakeExtensionValues[1])); // Extend intake

        // Optional: Short nudge if needed
        // TrajectoryActionBuilder nudgeBuilder = drive.actionBuilder(this.currentPose);
        // nudgeBuilder.forward(1); sequence.add(nudgeBuilder.build()); this.currentPose = nudgeBuilder.endPose;

        // 2. Secure Sample & Retract Intake
        sequence.add(new SetIntakeServosAction(Constants.intakeSampleReset)); // Close claw
        // IMPORTANT: Define Constants.INTAKE_RETRACT_TARGET_TICKS in your Constants.java
        sequence.add(new MoveIntakeExtensionPIDAction(Constants.intakeExtensionValues[0])); // Retract intake
        // Lift intake assembly to transfer height (if applicable)
        // IMPORTANT: Constants.intakeSampleReset[5] should be the LIFT target for transfer.
        sequence.add(new MoveLiftPDFLAction((int)Constants.liftExtensionValues[0]));


        // 3. Transfer: Deposit aligns, (Intake might open, Deposit grabs - simplified here)
        sequence.add(new SetDepositServosAction(Constants.outtakeSampleReset)); // Deposit servos align for transfer
        sequence.add(new SleepAction(0.75)); // Time for alignment/transfer

        // 4. Score
        sequence.add(new MoveLiftPDFLAction((int)Constants.liftExtensionValues[3])); // Lift to score height

        TrajectoryActionBuilder scoreMoveBuilder = drive.actionBuilder(this.currentPose);
        // EXAMPLE Score Pose - Tune this based on where the robot is after intake/lift for transfer
        scoreMoveBuilder.lineToLinearHeading(new Pose2d(this.currentPose.position.x + 15, this.currentPose.position.y + 10, Math.toRadians(45)));
        sequence.add(scoreMoveBuilder.build());
        this.currentPose = scoreMoveBuilder.endPose; // Update pose to scoring location

        sequence.add(new SetDepositServosAction(Constants.outtakeSampleDrop)); // Deposit servos drop pixel
        sequence.add(new SleepAction(1.0)); // Wait for drop

        // 5. Retract after scoring
        sequence.add(new SetDepositServosAction(Constants.outtakeSampleReset));
        sequence.add(new MoveLiftPDFLAction((int)Constants.outtakeSampleReset[4])); // Retract lift
        // Optional: Retract intake extension fully if not already
        // sequence.add(new MoveIntakeExtensionPIDAction(Constants.INTAKE_FULLY_RETRACTED_TICKS));
    }

    private void initializeVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionPipeline = new ContrastAndROI();
        visionPipeline.setInspectionROI(inspectionROI);
        webcam.setPipeline(visionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT); isWebcamStreaming = true; }
            @Override public void onError(int errorCode) { isWebcamStreaming = false; telemetry.addData("Error", "Webcam Error: " + errorCode); }
        });
    }

    private void performScan() {
        if (!isWebcamStreaming || !opModeIsActive()) return;
        telemetry.addLine("Performing scan...");
        telemetry.update();
        scanTimer.reset();
        detectedPosition = DetectionResult.NONE;
        while (opModeIsActive() && !isStopRequested() && scanTimer.seconds() < SCAN_DURATION_SEC) {
            List<Rect> currentDetections = visionPipeline.getDetectedRectangles();
            if (!currentDetections.isEmpty()) {
                Rect firstObject = currentDetections.get(0);
                double objectCenterXInFullFrame = firstObject.x + firstObject.width / 2.0;
                double roiThirdWidth = inspectionROI.width / 3.0;
                double roiLeftBoundary = inspectionROI.x + roiThirdWidth;
                double roiCenterBoundary = inspectionROI.x + (2.0 * roiThirdWidth);
                if (objectCenterXInFullFrame < roiLeftBoundary) detectedPosition = DetectionResult.LEFT;
                else if (objectCenterXInFullFrame < roiCenterBoundary) detectedPosition = DetectionResult.CENTER;
                else if (objectCenterXInFullFrame < inspectionROI.x + inspectionROI.width) detectedPosition = DetectionResult.RIGHT;
                else detectedPosition = DetectionResult.NONE;
            } else {
                detectedPosition = DetectionResult.NONE;
            }
            sleep(50);
        }
        telemetry.addData("Scan complete. Result", detectedPosition);
        telemetry.update();
    }

    private void cleanup() {
        if (webcam != null) {
            if (isWebcamStreaming) webcam.stopStreaming();
            webcam.closeCameraDevice();
            isWebcamStreaming = false;
        }
        if (visionPipeline != null) {
            try { visionPipeline.getClass().getMethod("cleanup").invoke(visionPipeline); }
            catch (Exception e) { /* Ignored */ }
        }
        if (lifts != null) {
            lifts.enablePDControl(false);
            lifts.stopSlides();
        }
        if (intake != null) {
            intake.setMotorPower(0); // Stop intake extension motor
        }
        telemetry.addLine("Cleanup complete.");
        telemetry.update();
    }
}
