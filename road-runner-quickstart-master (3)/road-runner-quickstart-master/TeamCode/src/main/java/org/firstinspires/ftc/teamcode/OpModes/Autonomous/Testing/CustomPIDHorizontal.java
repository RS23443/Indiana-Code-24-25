package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;


@Config
@TeleOp
public class CustomPIDHorizontal extends OpMode {
    public PIDController pidController;
    public Intake intake;
    public double kP = 0.01, kI = 0.00, kD = 0.000, alpha = 0.075;
    public static double target = 300;
    public DcMotorEx horizontalExtension;


    @Override
    public void init() {
        // Initialize the PDFL controller
        pidController = new PIDController(kP, kD, kI, alpha);


        // Configure motors
        intake = new Intake(hardwareMap);
        intake.setServoPosition(1,0.66);
        intake.setServoPosition(2,0.34);
        intake.setServoPosition(3,0.3);
        intake.setServoPosition(4,0.7);
        intake.setServoPosition(5,0.45);
        horizontalExtension = hardwareMap.get(DcMotorEx.class, "horizontal_extension");
        //horizontalExtension.setDirection(DcMotorEx.Direction.REVERSE);


        // Configure telemetry with FTC Dashboard
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void loop() {
        double power = pidController.compute(horizontalExtension.getCurrentPosition(),target);
        horizontalExtension.setPower(power);
        telemetry.addData("PWR", power);

    }
}
