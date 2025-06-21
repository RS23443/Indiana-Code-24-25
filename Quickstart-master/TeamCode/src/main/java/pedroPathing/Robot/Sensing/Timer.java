package pedroPathing.Robot.Sensing;

public class Timer {
    private long startTime;
    private long elapsedTime;
    private boolean running;

    // Constructor initializes the timer
    public Timer() {
        resetTimer();
    }

    // Starts or resumes the timer
    public void startTimer() {
        if (!running) {
            startTime = System.currentTimeMillis() - elapsedTime;
            running = true;
        }
    }

    // Stops the timer
    public void stopTimer() {
        if (running) {
            elapsedTime = System.currentTimeMillis() - startTime;
            running = false;
        }
    }

    // Resets the timer
    public void resetTimer() {
        startTime = System.currentTimeMillis();
        elapsedTime = 0;
        running = true;
    }

    // Gets the elapsed time in milliseconds
    public long getTimeMillis() {
        if (running) {
            return System.currentTimeMillis() - startTime;
        } else {
            return elapsedTime;
        }
    }

    // Gets the elapsed time in seconds
    public long getTimeSeconds() {
        return getTimeMillis() / 1000;
    }
    public boolean isRunning() {
        return running;
    }
}
