package pedroPathing.Robot;

public class PIDController {
    private double kP, kI, kD, alpha;
    private double integralSum = 0;
    private double lastError = 0;
    private double prevFilteredDerivative = 0;
    private long lastTime;

    public PIDController(double kP, double kI, double kD, double alpha) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.alpha = alpha;
        this.lastTime = System.nanoTime();
    }


    public double compute(double measurement, double reference) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;

        double error = reference - measurement;
        integralSum += error * dt;

        // Apply simple anti-windup
        integralSum = Math.max(-0.25, Math.min(0.25, integralSum));

        double derivative = (error - lastError) / dt;
        double filteredDerivative = alpha * prevFilteredDerivative + (1 - alpha) * derivative;

        lastError = error;
        prevFilteredDerivative = filteredDerivative;

        return (kP * error) + (kI * integralSum) + (kD * filteredDerivative);
    }
}
