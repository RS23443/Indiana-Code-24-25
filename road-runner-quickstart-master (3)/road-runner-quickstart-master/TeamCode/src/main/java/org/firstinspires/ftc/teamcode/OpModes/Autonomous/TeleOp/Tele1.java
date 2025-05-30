package org.firstinspires.ftc.teamcode.OpModes.Autonomous.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Commands.DepositDropPositionCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.DepositResetCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.IntakeActiveCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.IntakeGrabCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.PDFLCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.SampleSwitchCommand;
import org.firstinspires.ftc.teamcode.Robot.Systems.*;
import org.firstinspires.ftc.teamcode.Robot.Sensing.*;


@TeleOp(name = "TeleOp - Main")
public class Tele1  extends OpMode {
    //hardware
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public CRServo leftwinch, rightwinch;
    public Lifts lifts;
    public Intake intake;
    public Deposit deposit;
    public VoltageSensor voltageSensor;
    public LimelightBaseSystem limelight;
    public Timer voltageSensorTimer;

    //Variables
    public boolean isIntakeReadyToTransfer = false;
    public boolean isOuttakeClawOpenForDeposit = false;
    private boolean lastLBState = false;
    private boolean isSleeping = false;
    public double speed = 1.0;
    private double initialVoltage;
    private long sleepEndTime = 0;
    private int sleepState = 0;
    public int lb_counter = 1;




    @Override
    public void init() {
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
        leftwinch = hardwareMap.get(CRServo.class,"left_winch");
        rightwinch = hardwareMap.get(CRServo.class,"right_winch");

        limelight = new LimelightBaseSystem(hardwareMap, "limelight");
        limelight.startLimelight();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        initialVoltage = voltageSensor.getVoltage();
        voltageSensorTimer = new Timer();

        lifts = new Lifts(hardwareMap,voltageSensor.getVoltage());
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        gp2_lb();
        double time = voltageSensorTimer.getTimeMillis();
        double voltage = 0;
        // need to figure out what to do with this
        if (time % 100 == 0) {
            double  newVoltage = voltageSensor.getVoltage();
            if (newVoltage >= (initialVoltage - 1.0)){
                voltage = newVoltage;
            }
        }
        //auto extends winch once end game starts
        if (120*1e3 - voltageSensorTimer.getTimeMillis() < 30*1e3){
            rightwinch.setPower(1);
            leftwinch.setPower(1);
        }

        if (gamepad1.a) {
                intake.setMotor(595);
        }

        if (intake.getExtensionPosition() > 300) {
            new IntakeActiveCommand(intake);
        }


        if(gamepad1.b){
            new IntakeGrabCommand(intake);
            isIntakeReadyToTransfer = true;
        }
        // experimental stages
        /*if(isIntakeReadyToTransfer){
            limelight.FetchResults(); // need to change this, but can only do that when transfer is finalized
            if(limelight.angleOfBlock()>3000){
                new SequentialCommandGroup(new SampleSwitchCommand(intake,deposit, lifts),new WaitCommand(150), new DepositDropPositionCommand(deposit));
            }
        }*/

        if(/*(lifts.getTopMotorData()[0] >= 50 && lifts.getTopMotorData()[1] == 0 && isIntakeReadyToTransfer) ||*/  gamepad1.x){
            new SequentialCommandGroup(new SampleSwitchCommand(intake,deposit, lifts),new WaitCommand(150), new DepositDropPositionCommand(deposit));
        }

        if(gamepad1.y){
            new DepositResetCommand(deposit, lifts);
        }

        if(gamepad1.right_bumper){
            rightwinch.setPower(-1);
            leftwinch.setPower(-1);
        }


        double rx = gamepad1.right_stick_x;
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + rx + x) / denominator * speed);
        leftBack.setPower((y - x + rx) / denominator * speed);
        rightFront.setPower((y - rx - x) / denominator * speed);
        rightBack.setPower((y - rx + x) / denominator * speed);


    }
    public void gp2_lb() {
        boolean currentLBState = gamepad2.left_bumper;
        if (currentLBState && !lastLBState) {
            if (lb_counter == 1) {
                new PDFLCommand(lifts,1000).schedule();
                lb_counter = 2;
            } else {
                new PDFLCommand(lifts,30).schedule();
                lb_counter = 1;
            }
        }
        lastLBState = currentLBState;
    }
}
