package pedroPathing.Autonomous.Testing.Camera; // Adjust this package to your project structure

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import pedroPathing.Robot.Camera.Pipelines.ContrastAndROI;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;

/**
 * This OpMode demonstrates an incremental horizontal scan for yellow blocks
 * using the ContrastAndROI pipeline in a LinearOpMode, similar to the original
 * Road Runner Action structure but without using FTCLib commands.
 * It will extend the intake incrementally, scan at each position, and retract
 * if a yellow block is found.
 */
@Autonomous(name = "Incremental Yellow Scan (Linear)", group = "Autonomous")
public class IncrementalYellowScan extends LinearOpMode {

    // Vision-related variables
    OpenCvCamera webcam;
    ContrastAndROI visionPipeline;
    public DetectionResult detectedPosition = DetectionResult.NONE;
    int cameraWidth = 1280;
    int cameraHeight = 720;
    Rect inspectionROI; // This ROI will be used for the entire camera frame

    private volatile boolean isWebcamStreaming = false;
    private static final double SCAN_DURATION_SEC = 2.5; // Duration to scan at each position
    private ElapsedTime scanTimer = new ElapsedTime(); // For performScan() method
    private ElapsedTime intakeSettleTimer = new ElapsedTime(); // For waiting for intake to settle

    // Robot subsystem
    public Intake intake;

    // Enum for detection results
    public enum DetectionResult { LEFT, CENTER, RIGHT, NONE }

    // Global flag to indicate if yellow has been found and acted upon
    public boolean yellowBlockFoundAndProcessed = false;

    // Constants for Intake Extension and Scan Logic
    // IMPORTANT: Ensure these Constants.intakeExtensionValues indices are correct for your setup
    private final int INITIAL_SCAN_START_TICKS = Constants.intakeExtensionValues[0] + 50; // Start slightly extended
    private final int INTAKE_INCREMENT_TICKS = Constants.intakeExtensionValues[2];
    private final int MAX_INTAKE_EXTENSION_TICKS = Constants.intakeExtensionValues[1];
    private final int RETRACT_ON_DETECTION_AMOUNT_TICKS = Constants.intakeExtensionValues[3];
    private final double VELOCITY_ZERO_THRESHOLD = Constants.intakeExtensionValues[4];

    // Define a maximum number of scan steps to prevent infinite loop
    private static final int MAX_SCAN_STEPS_LIMIT = 20; // Safety limit for dynamically generated positions

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Incremental Yellow Scan OpMode (Linear)...");
        telemetry.update();

        // Initialize Subsystems
        intake = new Intake(hardwareMap);

        // Define ROI for vision to cover the entire camera frame for full field scan
        inspectionROI = new Rect(0, 0, cameraWidth, cameraHeight);

        // Initialize vision asynchronously
        initializeVision();

        telemetry.addData("Status", "Initialized. Waiting for Start...");
        telemetry.addData("Pipeline ROI", "x: %d, y: %d, w: %d, h: %d",
                inspectionROI.x, inspectionROI.y, inspectionROI.width, inspectionROI.height);
        telemetry.update();

        // Initial servo positions
        intake.setServoPosition(6, 0.7); // claw close
        intake.setServoPosition(1, 0.6); // servo 1
        intake.setServoPosition(2, 0.4); // servo 2
        sleep(200);
        intake.setServoPosition(3, 0.25); // servo 3
        intake.setServoPosition(4, 0.75); // servo 4
        sleep(200);

        waitForStart();

        if (isStopRequested()) {
            cleanup();
            return;
        }

        if (!isWebcamStreaming) {
            telemetry.addLine("Webcam did not start. Aborting incremental scan sequence.");
            telemetry.update();
            sleep(3000);
            cleanup();
            return;
        }

        // --- Dynamically generate scan positions based on constants ---
        List<Integer> intakeScanPositions = new ArrayList<>();
        int currentGeneratedPos = INITIAL_SCAN_START_TICKS;

        // Generate positions incrementally until MAX_INTAKE_EXTENSION_TICKS is reached or exceeded
        // Also add a safety limit based on MAX_SCAN_STEPS_LIMIT to prevent excessively long lists
        while (currentGeneratedPos <= MAX_INTAKE_EXTENSION_TICKS && intakeScanPositions.size() < MAX_SCAN_STEPS_LIMIT) {
            intakeScanPositions.add(currentGeneratedPos);
            currentGeneratedPos += INTAKE_INCREMENT_TICKS;
        }

        // Ensure the absolute MAX_INTAKE_EXTENSION_TICKS is included as the last position if not already present
        // This handles cases where the last increment overshoots the max, or if max is the only position.
        if (intakeScanPositions.isEmpty() || intakeScanPositions.get(intakeScanPositions.size() - 1) < MAX_INTAKE_EXTENSION_TICKS) {
            intakeScanPositions.add(MAX_INTAKE_EXTENSION_TICKS);
        }
        // --- End dynamic generation ---

        telemetry.addData("Generated Scan Positions", intakeScanPositions.size());
        telemetry.update();
        sleep(500); // Small delay to show generated positions telemetry

        // Iterate through each predefined scan position
        for (int i = 0; i < intakeScanPositions.size() && opModeIsActive() && !isStopRequested(); i++) {
            // Check if yellow has already been found in a previous iteration
            if (yellowBlockFoundAndProcessed) {
                telemetry.addLine("Yellow already found, skipping remaining scan steps.");
                break; // Exit the loop if yellow was found
            }

            int targetPosition = intakeScanPositions.get(i);

            telemetry.addData("Scan Step", (i + 1) + " / " + intakeScanPositions.size());
            telemetry.addData("Moving Intake To", targetPosition);
            telemetry.update();

            // Move intake to the target position
            intake.setMotor(targetPosition);

            // Wait until intake settles at the new position
            telemetry.addLine("Waiting for intake to settle...");
            intakeSettleTimer.reset();
            while (opModeIsActive() && !isStopRequested() &&
                    Math.abs(intake.getHorizontalExtensionVelocity()) >= VELOCITY_ZERO_THRESHOLD &&
                    intakeSettleTimer.milliseconds() < 1500) { // Max 1.5s wait for settle
                telemetry.addData("Current Intake Velocity", String.format("%.2f", intake.getHorizontalExtensionVelocity()));
                telemetry.addData("Waiting for Settle", String.format("%.2f s", intakeSettleTimer.seconds()));
                telemetry.update();
                sleep(20);
            }
            // IMPORTANT: Explicitly stop the intake motor after it has settled
            // This is crucial to prevent "shooting out" if setMotor doesn't hold position
            intake.setMotorPower(0);
            telemetry.addLine("Intake settled. Performing scan.");

            // Perform the scan (this method contains internal sleep calls for SCAN_DURATION_SEC)
            performScan();

            // After performScan completes, detectedPosition is updated
            if (detectedPosition != DetectionResult.NONE) {
                yellowBlockFoundAndProcessed = true; // Set flag if yellow is found
                telemetry.addData("Detection", "Yellow detected at pos: " + intake.getExtensionPosition());
                // The loop's primary condition (!yellowBlockFoundAndProcessed) will cause it to exit naturally.
            } else {
                telemetry.addLine("No yellow detected at this position.");
            }

            // Small delay before moving to the next scan position, unless yellow was found or we're done.
            // This delay is after the scan, before the next movement command.
            if (!yellowBlockFoundAndProcessed && opModeIsActive() && !isStopRequested() && i < intakeScanPositions.size() - 1) {
                sleep(500); // Wait for 0.5 seconds before the next incremental move.
            }
        } // End for loop

        // Final conditional action: Retract and grab if yellow was found during the scan
        if (opModeIsActive() && yellowBlockFoundAndProcessed) {
            int retractToTarget = Math.max(0, intake.getExtensionPosition() - RETRACT_ON_DETECTION_AMOUNT_TICKS);
            telemetry.addData("Final Action", "Yellow Found: Retracting and Grabbing").addData("Retract To", retractToTarget);

            intake.setMotor(retractToTarget);
            // Wait for retraction to complete (simplified wait here, could be more robust)
            long retractStartTime = System.currentTimeMillis();
            while (opModeIsActive() && !isStopRequested() &&
                    Math.abs(intake.getExtensionPosition() - retractToTarget) > 20 && // Basic position check
                    System.currentTimeMillis() - retractStartTime < 3000) { // Max 3s for retraction
                telemetry.addData("Retracting...", "Current Pos: " + intake.getExtensionPosition());
                telemetry.update();
                sleep(20);
            }
            intake.setMotorPower(0); // Stop motor if using PID that holds position

            // Servo actions after retraction
            intake.setServoPosition(5,0.45); // This would be the equivalent of Intake45Degrees
            sleep(200);
            intake.setServoPosition(6,0.48); // This would be the equivalent of IntakeGrabCommand
            sleep(200);

            telemetry.addLine("Intake retracted and grabbed. Incremental scan complete.");
        } else {
            telemetry.addLine("No yellow found throughout scan, skipping final retraction/grab sequence.");
        }

        telemetry.addLine("OpMode finished.");
        telemetry.update();
        sleep(3000); // Keep final telemetry visible for a few seconds

        cleanup(); // Final cleanup
    }

    // No dispose method, as per request. Cleanup is called directly.
    private void cleanup() {
        telemetry.addLine("OpMode cleaning up...");
        if (webcam != null) {
            if (isWebcamStreaming) webcam.stopStreaming();
            webcam.closeCameraDevice();
            isWebcamStreaming = false;
            telemetry.addLine("Webcam closed.");
        }
        if (visionPipeline != null) {
            try {
                java.lang.reflect.Method cleanupMethod = visionPipeline.getClass().getMethod("cleanup");
                if (cleanupMethod != null) {
                    cleanupMethod.invoke(visionPipeline);
                    telemetry.addLine("Vision pipeline cleaned.");
                }
            } catch (NoSuchMethodException e) { /* No 'cleanup' method, that's fine */
            } catch (Exception e) {
                telemetry.addData("Error", "Exception cleaning vision pipeline: " + e.getMessage());
            }
        }
        if (intake != null) {
            intake.setMotorPower(0); // Ensure motor is stopped
            telemetry.addLine("Intake motor stopped.");
        }
        telemetry.addLine("Cleanup complete.");
        telemetry.update();
    }

    private void initializeVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionPipeline = new ContrastAndROI();
        visionPipeline.setInspectionROI(inspectionROI); // Set the full frame ROI
        webcam.setPipeline(visionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                isWebcamStreaming = true;
                telemetry.addLine("Webcam streaming started.");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                isWebcamStreaming = false;
                telemetry.addData("Error", "Webcam Error: " + errorCode);
                telemetry.update();
            }
        });
    }

    // Re-integrated performScan method
    private void performScan() {
        if (!isWebcamStreaming || !opModeIsActive()) {
            detectedPosition = DetectionResult.NONE;
            telemetry.addLine("Scan skipped: Webcam not streaming or OpMode inactive.");
            telemetry.update();
            return;
        }
        telemetry.addLine("Performing scan...");
        telemetry.update();
        sleep(100); // Small blocking sleep to allow telemetry to update

        scanTimer.reset();
        detectedPosition = DetectionResult.NONE; // Reset for each scan to avoid carrying over old detections
        DetectionResult frameDetection = DetectionResult.NONE;

        while (opModeIsActive() && scanTimer.seconds() < SCAN_DURATION_SEC) {
            List<Rect> currentDetections = visionPipeline.getDetectedRectangles();
            if (!currentDetections.isEmpty()) {
                Rect firstObject = currentDetections.get(0);
                double objectCenterXInFullFrame = firstObject.x + firstObject.width / 2.0;
                double frameThirdWidth = (double)cameraWidth / 3.0;
                double leftBoundary = frameThirdWidth;
                double centerBoundary = 2.0 * frameThirdWidth;

                if (objectCenterXInFullFrame < leftBoundary) frameDetection = DetectionResult.LEFT;
                else if (objectCenterXInFullFrame < centerBoundary) frameDetection = DetectionResult.CENTER;
                else frameDetection = DetectionResult.RIGHT;
            } else {
                frameDetection = DetectionResult.NONE;
            }
            if (frameDetection != DetectionResult.NONE) {
                detectedPosition = frameDetection; // Update the overall detectedPosition
            }

            telemetry.addData("Scanning...", String.format("%.1fs / %.1fs", scanTimer.seconds(), SCAN_DURATION_SEC));
            telemetry.addData("Frame Detection", frameDetection);
            telemetry.addData("Overall Scan Detection", detectedPosition);
            telemetry.update();
            sleep(50); // Small blocking sleep to allow pipeline to process
        }
        telemetry.addData("Scan complete. Final Result for this position", detectedPosition);
        telemetry.update();
    }
}
