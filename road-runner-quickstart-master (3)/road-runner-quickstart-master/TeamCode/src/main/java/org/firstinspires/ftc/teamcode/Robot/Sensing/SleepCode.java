package org.firstinspires.ftc.teamcode.Robot.Sensing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class SleepCode {
    public static MultipleTelemetry telemetry;
    private long endTime;

    /**
     * Constructor to set up the sleep duration.
     *
     * @param milliseconds The amount of time to wait
     */
    public SleepCode(long milliseconds) {
        this.endTime = System.currentTimeMillis() + milliseconds;
    }

    /**
     * Checks if the sleep time has elapsed.
     *
     * @return True if the sleep time is over, false otherwise
     */
    public boolean isFinished() {
        long remainingTime = endTime - System.currentTimeMillis();
        if (telemetry != null) {
            telemetry.addData("Time until Next Action", Math.max(remainingTime, 0));
            telemetry.update();
        }
        return remainingTime <= 0;
    }
}
