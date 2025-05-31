package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines.ContrastAndROI;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Set;

@Config
@Autonomous(name = "Auto Version 1", group = "Autonomous")
public class AutoV1 extends LinearOpMode {
    public class Attachements{
        private Lifts lifts;
        private Intake intake;
        private Deposit deposit;
        private OpenCvCamera webcam;
        private ContrastAndROI visionPipeline;
        public int cameraWidth = 1280;
        public int cameraHeight = 720;
        public Rect inspectionROI = new Rect(0,0,1280,720);

        private volatile boolean isWebcamStreaming = false;
        private static final double SCAN_DURATION_SEC = 0.5;
        private ElapsedTime scanTimer = new ElapsedTime();

        public Attachements(HardwareMap hardwareMap, double voltage, Telemetry telemetry){
            lifts = new Lifts(hardwareMap, voltage);
            intake = new Intake(hardwareMap);
            deposit = new Deposit(hardwareMap);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            visionPipeline = new ContrastAndROI();
            visionPipeline.setInspectionROI(inspectionROI);
            webcam.setPipeline(visionPipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override public void onOpened() {
                    webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                    isWebcamStreaming = true;
                }
                @Override public void onError(int errorCode) {
                    isWebcamStreaming = false;
                    telemetry.addData("Error", "Webcam Error: " + errorCode);
                }
            });
        }

        @SuppressLint("NotConstructor")
         public class PDFLAction implements Action {
            private final Lifts liftSubsystem;
            private final int targetPosition;
            private boolean initialized = false;
            private static final int POSITION_TOLERANCE = 35;
            public PDFLAction(Lifts liftSubsystem, int targetPosition) {
                this.liftSubsystem = liftSubsystem;
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // Initialize the lift for the movement
                    liftSubsystem.setTarget(targetPosition);
                    liftSubsystem.enablePDControl(true); // Ensure PD control is active
                    liftSubsystem.slidePDFL(targetPosition); // Specific initialization for this type of move
                    initialized = true;
                    packet.put("LiftActionStatus", "Initialized to " + targetPosition);
                }

                liftSubsystem.updatePDControl();

                // Get current state from subsystem (values updated in Lifts.periodic())
                int currentLiftPos = (int) liftSubsystem.getTopMotorData()[0];
                packet.put("Lift Target", targetPosition);
                packet.put("Lift Current Pos", currentLiftPos);
                boolean isPositionReached = Math.abs(currentLiftPos - targetPosition) < POSITION_TOLERANCE;
                boolean isMotorStopped = Math.abs(liftSubsystem.getTopMotorData()[1]) <= 20.0;
                packet.put("Lift At Position", isPositionReached);
                boolean isDone = isPositionReached && isMotorStopped;

                if (isDone) {
                    packet.put("LiftActionStatus", "Completed");
                    liftSubsystem.enablePDControl(false);
                    liftSubsystem.runslides(liftSubsystem.DynamicKF());
                    initialized = false;
                    return false;
                }

                return true;
            }
        }

        public class IntakeExtension implements Action {
            private final Intake intake;
            private final int target;

            private boolean initialized = false;
            private boolean atPosition = false;
            private boolean motorStopped = false;
            private int initialPosition;
            private int direction;

            private ElapsedTime timer = new ElapsedTime(); // Timer for timeout
            private static final double TIMEOUT_SECONDS = 1.5; // Timeout duration

            private static final int POSITION_TOLERANCE = 10;
            private static final double VELOCITY_THRESHOLD = 10.0;

            public IntakeExtension(Intake intake, int target) {
                this.intake = intake;
                this.target = target;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialPosition = intake.getExtensionPosition();
                    direction = target - initialPosition;
                    // intake.setMotor(target); // You re-added this in your version, keep if necessary
                    timer.reset(); // Reset timer at the start of the action
                    initialized = true;
                    atPosition = false;
                    motorStopped = false;
                }

                intake.setMotor(target); // You re-added this in your version.
                // If this is essential every loop, keep it.
                // Ideally, for PID, it's set once.

                int position = intake.getExtensionPosition();
                double velocity = intake.getHorizontalExtensionVelocity();

                telemetryPacket.put("IntakeExt Target", target);
                telemetryPacket.put("IntakeExt Position", position);
                telemetryPacket.put("IntakeExt Velocity", velocity);
                telemetryPacket.put("IntakeExt Direction", direction);
                telemetryPacket.put("IntakeExt Time Elapsed (s)", String.format("%.2f", timer.seconds()));


                if (direction < 0) { // Moving towards a numerically smaller position
                    if (position <= target + POSITION_TOLERANCE) {
                        atPosition = true;
                    } else {
                        atPosition = false;
                    }
                } else { // Moving towards a numerically larger position (or target is the same as initial)
                    if (position >= target - POSITION_TOLERANCE) {
                        atPosition = true;
                    } else {
                        atPosition = false;
                    }
                }

                if (Math.abs(velocity) < VELOCITY_THRESHOLD) {
                    motorStopped = true;
                } else {
                    motorStopped = false;
                }

                telemetryPacket.put("IntakeExt AtPosition", atPosition);
                telemetryPacket.put("IntakeExt MotorStopped", motorStopped);

                boolean isDoneByCompletion = atPosition && motorStopped;
                boolean isTimedOut = timer.seconds() >= TIMEOUT_SECONDS;

                if (isDoneByCompletion || isTimedOut) {
                    if (isTimedOut) {
                        telemetryPacket.put("IntakeExt Status", "Timed Out");
                        intake.setMotorPower(0); // Safely stop motor on timeout
                    } else {
                        telemetryPacket.put("IntakeExt Status", "Completed");
                        if (direction < 0) {
                            intake.setMotorPower(0);
                        } else {
                            if (target > 0 || direction != 0) {
                                intake.setMotorPower(0.075);
                            } else {
                                intake.setMotorPower(0);
                            }
                        }
                    }
                    initialized = false; // Reset for potential re-run
                    return false; // Action is complete (either by success or timeout)
                }

                telemetryPacket.put("IntakeExt Status", "Running");
                return true;
            }
        }

            public class Deposit_Reset implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    deposit.setServoPosition(4, 0.8);
                    new SleepAction(0.2);
                    deposit.setServoPosition(3, 0.98);
                    deposit.setServoPosition(1, 0.5);
                    deposit.setServoPosition(2, 0.5);
                    return false;
                }
            }

            public class Deposit_Grab implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]);
                    new SleepAction(0.15);
                    deposit.setServoPosition(1, Constants.outtakeSampleDrop[0]);
                    deposit.setServoPosition(2, Constants.outtakeSampleDrop[1]);
                    deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);
                    return false;
                }
            }

            public class BlockSwitch implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]);
                    new SleepAction(0.15);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                    return false;
                }
            }

            public class Intake_Active implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setServoPosition(1, 0.8);
                    intake.setServoPosition(2, 0.2);
                    new SleepAction(0.2);
                    intake.setServoPosition(3, Constants.intakeActive[2]);
                    intake.setServoPosition(4, Constants.intakeActive[3]);
                    new SleepAction(0.2);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                    return false;
                }
            }

            public class Intake_Grab implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setServoPosition(1, 0.6);
                    intake.setServoPosition(2, 0.4);
                    new SleepAction(0.2);
                    intake.setServoPosition(3, 0.21);
                    intake.setServoPosition(4, 0.79);
                    return false;
                }
            }

        public class Intake_PreGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setServoPosition(1, 0.5);
                intake.setServoPosition(2, 0.5);
                new SleepAction(0.2);
                intake.setServoPosition(3, 0.79);
                intake.setServoPosition(4, 0.21);
                return false;
            }
        }

            public class Intake45Degrees implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setServoPosition(1, Constants.intakeActive[0]);
                    intake.setServoPosition(2, Constants.intakeActive[1]);
                    new SleepAction(0.2);
                    intake.setServoPosition(3, Constants.intakeHorizontalSpin[2]);
                    intake.setServoPosition(4, Constants.intakeHorizontalSpin[3]);
                    new SleepAction(0.2);
                    intake.setServoPosition(6, Constants.intakeActive[4]);
                    return false;
                }
            }

            public Action DepositOpen() {
                return new InstantAction(new InstantFunction() {
                    @Override
                    public void run() {
                        deposit.setServoPosition(4, Constants.outtakeSampleReset[3]);
                    }
                });
            }

            public Action intakeDropDown() {
                return new InstantAction(new InstantFunction() {
                    @Override
                    public void run() {
                        intake.setServoPosition(1, 0.9);
                        intake.setServoPosition(2, 0.1);
                    }
                });
            }

            public Action IntakeClose() {
                return new InstantAction(new InstantFunction() {
                    @Override
                    public void run() {
                        intake.setServoPosition(6, 0.44);
                    }
                });
            }

            public Action IntakeTransferClaw(){
            return  new InstantAction(new InstantFunction() {
                @Override
                public void run() {
                    intake.setServoPosition(6,0.45);
                }
            });
            }


            public Action PDFLAction(int target) {
                return new Attachements.PDFLAction(lifts, target);
            }

            public Action IntakePID(int target) {
                return new IntakeExtension(intake, target);
            }

            public Action Deposit_Reset() {
                return new Attachements.Deposit_Reset();
            }

            public Action Deposit_Grab() {
                return new Attachements.Deposit_Grab();
            }

            public Action BlockSwitch() {
                return new Attachements.BlockSwitch();
            }

            public Action Intake_Active() {
                return new Attachements.Intake_Active();
            }

            public Action Intake_Grab() {
                return new Attachements.Intake_Grab();
            }

            public Action Intake_PreGrab(){
            return new Attachements.Intake_PreGrab();
            }

            public Action Intake45Degrees() {
                return new Attachements.Intake45Degrees();
            }
    }
    public MecanumDrive drivetrain;
    public Pose2d startpose = new Pose2d(0,-1,0);
    public VoltageSensor voltageSensor;
    public Attachements attachements;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap,startpose);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        attachements = new Attachements(hardwareMap,voltageSensor.getVoltage(),telemetry);

        TrajectoryActionBuilder preload = drivetrain.actionBuilder(startpose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-15.5,6.5,Math.toRadians(45)),0)
                .endTrajectory();
        TrajectoryActionBuilder s2 = preload.endTrajectory().fresh()
                .setTangent(45)
                .turnTo(Math.toRadians(69))
                .setTangent(90)
                .lineToYConstantHeading(10.5)
                .endTrajectory();
        TrajectoryActionBuilder scores2 = s2.endTrajectory().fresh()
                .setTangent(0)
                .turnTo(Math.toRadians(45))
                .setTangent(90)
                .lineToYConstantHeading(6)
                .endTrajectory();
        TrajectoryActionBuilder s3 = scores2.endTrajectory().fresh()
                .setTangent(45)
                .turnTo(Math.toRadians(92.5))
                .setTangent(90)
                .lineToYConstantHeading(8.25)
                .endTrajectory();
        TrajectoryActionBuilder s3score = s3.endTrajectory().fresh()
                .setTangent(0)
                .turnTo(Math.toRadians(54))
                .setTangent(90)
                .lineToYConstantHeading(6)
                .endTrajectory();

        Action traj1 = preload.build();
        Action traj2 = s2.build();
        Action traj3 = scores2.build();
        Action traj4 = s3.build();
        Action traj5 = s3score.build();
        ParallelAction p1 = new ParallelAction(traj1,attachements.PDFLAction(1300),attachements.Deposit_Grab());
        ParallelAction liftsPlusIntake = new ParallelAction(attachements.PDFLAction(-10),attachements.Deposit_Reset(), new SequentialAction(attachements.IntakePID(600),attachements.intakeDropDown(), new SleepAction(0.2),attachements.IntakeClose(),new SleepAction(0.5),attachements.Intake_Grab(), attachements.IntakePID(-5), attachements.IntakeTransferClaw()));
        ParallelAction dropextend = new ParallelAction(attachements.DepositOpen(), attachements.IntakePID(200), attachements.Intake_Active());
        ParallelAction p2 = new ParallelAction(traj2, attachements.PDFLAction(10));
        ParallelAction p3 = new ParallelAction(traj3,attachements.BlockSwitch());
        ParallelAction p4 = new ParallelAction(traj4, attachements.PDFLAction(10));
        ParallelAction p5 = new ParallelAction(traj5,attachements.BlockSwitch());
        ParallelAction liftsanddrop = new ParallelAction(attachements.PDFLAction(1400),attachements.Deposit_Grab());
        Actions.runBlocking(attachements.Deposit_Grab());
        Actions.runBlocking(attachements.Intake_Grab());

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        traj1, attachements.PDFLAction(1400),dropextend, new SleepAction(0.2),traj2,liftsPlusIntake, traj3,attachements.BlockSwitch(),liftsanddrop, attachements.Deposit_Grab(),new SleepAction(0.4),new ParallelAction(attachements.DepositOpen(), attachements.Intake_Active()),p4, new ParallelAction(attachements.PDFLAction(-5),attachements.Deposit_Reset(), new SequentialAction(attachements.IntakePID(600),attachements.intakeDropDown(), new SleepAction(0.2),attachements.IntakeClose(),new SleepAction(0.5),attachements.Intake_PreGrab(),new SleepAction(0.4),attachements.Intake_Grab(), attachements.IntakePID(-5)), new SleepAction(1)),traj5,attachements.BlockSwitch(),new ParallelAction(attachements.PDFLAction(1400),attachements.Deposit_Grab()),attachements.Deposit_Grab(),new SleepAction(0.4),new ParallelAction(attachements.DepositOpen(), attachements.IntakePID(200), attachements.Intake_Active()))
        );
    }
}

