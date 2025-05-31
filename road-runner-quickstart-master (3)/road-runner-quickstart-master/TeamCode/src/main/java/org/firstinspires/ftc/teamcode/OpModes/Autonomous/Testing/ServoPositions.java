package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPositions extends OpMode {
    private Servo lif;
    private Servo rif;
    private Servo rightdiffy;
    private Servo leftdiffy;
    private Servo claw;
    private Servo lof;
    private Servo rof;
    private Servo right_elbow;
    private Servo o_claw;
    @Override
    public void init() {
        lif = hardwareMap.get(Servo.class, "left_intake_flip");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        leftdiffy = hardwareMap.get(Servo.class, "left_differential");
        rightdiffy = hardwareMap.get(Servo.class, "right_differential");
        claw = hardwareMap.get(Servo.class, "intake_claw");
        lof = hardwareMap.get(Servo.class, "left_outtake_flip");
        rof = hardwareMap.get(Servo.class, "right_outtake_flip");
        right_elbow = hardwareMap.get(Servo.class, "right_elbow");
        o_claw = hardwareMap.get(Servo.class, "outtake_claw");

        telemetry = FtcDashboard.getInstance().getTelemetry();

    }

    @Override
    public void loop() {
        double lifpos = lif.getPosition();
        double rifpos = rif.getPosition();
        double ldpos = leftdiffy.getPosition();
        double rdpos = rightdiffy.getPosition();
        double icpos = claw.getPosition();
        double lofpos = lof.getPosition();
        double rofpos = rof.getPosition();
        double elbowpos = right_elbow.getPosition();
        double ocpos =o_claw.getPosition();

        telemetry.addData("lif pos", lifpos);
        telemetry.addData("rif pos", rifpos);
        telemetry.addData("leftdiffy pos", ldpos);
        telemetry.addData("rightdiffy pos", rdpos);
        telemetry.addData("intake claw pos", icpos);
        telemetry.addData("lof pos", lofpos);
        telemetry.addData("rof pos", rofpos);
        telemetry.addData("outtake elbow pos", elbowpos);
        telemetry.addData("outtake claw pos", ocpos);
    }
}
