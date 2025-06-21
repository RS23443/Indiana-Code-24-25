package pedroPathing.Autonomous.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Autonomous.ParellelExp;
import pedroPathing.Robot.Commands.DepositResetCommand;
import pedroPathing.Robot.Commands.IntakeExtensionControlCommand;
import pedroPathing.Robot.Commands.IntakeGrabCommand;
import pedroPathing.Robot.Commands.IntakeHorizontalSpinCommand;
import pedroPathing.Robot.Commands.PDFLCommand;
import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Sensing.LimelightBaseSystem;
import pedroPathing.Robot.Sensing.Timer;
import pedroPathing.Robot.Systems.Deposit;
import pedroPathing.Robot.Systems.Intake;
import pedroPathing.Robot.Systems.Lifts;


@TeleOp(name = "TeleOp - Spec Att 1")
public class SpecTele  extends LinearOpMode {
    //hardware
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public Lifts lifts;
    public Intake intake;
    public Deposit deposit;
    public VoltageSensor voltageSensor;
    public LimelightBaseSystem limelight;
    public Timer voltageSensorTimer;

    //Variables
    public boolean lastGamepad2LeftBumperState = false;
    public boolean  lastdPadDownStateG2 = false;
    int currentStateDpadG2 = 1;
    public double speed = 1.0;
    private double initialVoltage;
    LiftState2 currentStateG2 = LiftState2.RAISE;

    public enum LiftState2 {
        RAISE,
        LITTLEDEC,
        RESET
    }


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        limelight = new LimelightBaseSystem(hardwareMap, "limelight");
        limelight.startLimelight();

        initialVoltage = voltageSensor.getVoltage();
        voltageSensorTimer = new Timer();

        lifts = new Lifts(hardwareMap, initialVoltage);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        CommandScheduler.getInstance().reset();


        deposit.setServoPosition(1, Constants.outtakeSampleReset[0]);
        deposit.setServoPosition(2, Constants.outtakeSampleReset[1]);
        deposit.setServoPosition(3, Constants.outtakeSampleReset[2]);
        deposit.setServoPosition(4, Constants.outtakeSampleReset[3]);

        intake.setServoPosition(1, 0.8);
        intake.setServoPosition(2, 0.2);
        intake.setServoPosition(3, 0.5);
        intake.setServoPosition(4, 0.5);
        intake.setServoPosition(6, Constants.intakeActive[4]);

        waitForStart();
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            double rx = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + rx + x) / denominator * speed);
            leftBack.setPower((y - x + rx) / denominator * speed);
            rightFront.setPower((y - rx - x) / denominator * speed);
            rightBack.setPower((y - rx + x) / denominator * speed);

            if (gamepad2.a) {
                speed = 0.4;
                new SequentialCommandGroup(
                        new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[1] - 60),
                        new WaitCommand(200), // Wait before extension
                        new InstantCommand(() -> { // Step 1: Set first batch of servos
                            intake.setServoPosition(1, 0.80);
                            intake.setServoPosition(2, 0.2);
                        }),
                        new WaitCommand(200), // Wait
                        new InstantCommand(() -> { // Step 2: Set next batch of servos
                            intake.setServoPosition(3, Constants.intakeActive[2]);
                            intake.setServoPosition(4, Constants.intakeActive[3]);
                        }),
                        new WaitCommand(200), // Wait
                        new InstantCommand(() -> { // Step 3: Set final servo (e.g., claw)
                            intake.setServoPosition(6, Constants.intakeActive[4]); // Corrected index if for claw
                        })
                ).schedule();
            }

            if (gamepad2.b) {
                speed = 1.0;
                new SequentialCommandGroup(new IntakeGrabCommand(intake), new WaitCommand(200), new InstantCommand(() -> intake.setServoPosition(6, 0.45)), new IntakeExtensionControlCommand(intake, Constants.intakeExtensionValues[0])).schedule();
            }

            if (gamepad2.x) {
                new SequentialCommandGroup(
                        new InstantCommand(() -> { // Step 1: Set first batch of servos
                            intake.setServoPosition(1, 0.80);
                            intake.setServoPosition(2, 0.2);
                            intake.setServoPosition(3, Constants.intakeActive[2]);
                            intake.setServoPosition(4, Constants.intakeActive[3]);
                        }),
                        new WaitCommand(200), // Wait
                        new InstantCommand(() -> { // Step 3: Set final servo (e.g., claw)
                            intake.setServoPosition(6, Constants.intakeActive[4]); // Corrected index if for claw
                        })
                ).schedule();

            }

            if (gamepad2.y) {
                deposit.setServoPosition(1, Constants.outtakeSpecimenReset[0]);
                deposit.setServoPosition(2,Constants.outtakeSpecimenReset[1]);
                deposit.setServoPosition(3,Constants.outtakeSpecimenReset[2]);
                deposit.setServoPosition(4,Constants.outtakeSampleReset[3]);
            }

            boolean currentGamepad2LeftBumperState = gamepad2.left_bumper;
            // --- Edge Detector: This part is good, it runs the code only on the button press ---
            if (currentGamepad2LeftBumperState && !lastGamepad2LeftBumperState) {
                // Use a switch statement for cleaner state management
                switch (currentStateG2) {
                    case RAISE:
                        deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
                        deposit.setServoPosition(1,0.26);
                        deposit.setServoPosition(2,0.74);
                        deposit.setServoPosition(3,0.35);
                        // From INTAKE, move to the SCORING_HIGH position
                        new PDFLCommand(lifts, 1000).schedule();
                        currentStateG2 = LiftState2.LITTLEDEC;
                        telemetry.addLine("Lift State -> SCORING_HIGH");
                        break;

                    case LITTLEDEC:
                        new PDFLCommand(lifts, 500).schedule();
                        new WaitCommand(200).schedule();
                        deposit.setServoPosition(4, Constants.outtakeSpecimenReset[3] + 0.04);
                        currentStateG2 = LiftState2.RESET;
                        telemetry.addLine("Lift State -> INTAKE");
                        break;
                    case RESET:
                        deposit.setServoPosition(1, Constants.outtakeSpecimenReset[0]);
                        deposit.setServoPosition(2, Constants.outtakeSpecimenReset[1]);
                        deposit.setServoPosition(3, Constants.outtakeSpecimenReset[2]);
                        deposit.setServoPosition(4, Constants.outtakeSampleReset[3]);
                        new PDFLCommand(lifts, 0).schedule();
                        currentStateG2 = LiftState2.RAISE;
                        break;

                }
                // Update the last state for the next loop iteration
                lastGamepad2LeftBumperState = currentGamepad2LeftBumperState;


                boolean currentdPadGameStateG2 = gamepad2.dpad_down;
                if (currentdPadGameStateG2 && !lastdPadDownStateG2) {
                    if (currentStateDpadG2 == 1) {
                        new IntakeHorizontalSpinCommand(intake).schedule();
                        currentStateDpadG2 = 2;
                    } else {
                        new InstantCommand(() -> {
                            intake.setServoPosition(3, 0.5);
                            intake.setServoPosition(4, 0.5);
                        }).schedule();
                        currentStateDpadG2 = 1;
                    }
                }

                lastdPadDownStateG2 = currentdPadGameStateG2;
            }
        }

    }
}
