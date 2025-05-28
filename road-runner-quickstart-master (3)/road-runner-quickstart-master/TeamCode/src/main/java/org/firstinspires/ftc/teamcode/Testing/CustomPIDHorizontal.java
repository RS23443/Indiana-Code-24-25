package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.PIDController;


@Config
@TeleOp
public class CustomPIDHorizontal extends OpMode {
    public PIDController pidController;
    public static double kP, kI, kD, alpha = 0;
    public static double target = 0;
    public DcMotorEx horizontalExtension;


    @Override
    public void init() {
        // Initialize the PDFL controller
        pidController = new PIDController(kP, kD, kI, alpha);

        // Configure motors
        horizontalExtension = hardwareMap.get(DcMotorEx.class, "horizontal_extension");
        horizontalExtension.setDirection(DcMotorEx.Direction.REVERSE);


        // Configure telemetry with FTC Dashboard
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void loop() {
        double power = pidController.compute(horizontalExtension.getCurrentPosition(),target);
        horizontalExtension.setPower(power);

    }
}
