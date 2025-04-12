package org.firstinspires.ftc.teamcode.motionProfile;

public class MotionProfile {
    // Start and target positions
    public double startX, startY;
    public double targetX, targetY;

    // Motion parameters
    public double maxVelocity;
    public double maxAcceleration;

    // Profile values
    public double totalDistance;         // Total distance from start to target
    public double timeToMaxVelocity;     // Time to accelerate to peak velocity
    public double cruiseTime;            // Duration of constant (peak) velocity phase
    public double totalTime;             // Total time for the complete profile
    public double peakVelocity;          // The maximum velocity achieved in this profile

    // Dynamic fields updated during profile execution
    public double timeElapsed;           // Time elapsed since profile start (seconds)
    public int phase;                    // 0: accelerating, 1: cruising, 2: decelerating, 3: complete

    // Internal timer
    private long startTime;              // Start time in milliseconds

    /**
     * Constructs a MotionProfileRedux from a start position to a target position.
     */
    public MotionProfile(double startX, double startY, double targetX, double targetY, double maxVelocity, double maxAcceleration) {
        this.startX = startX;
        this.startY = startY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        // Compute the straight-line distance between start and target.
        totalDistance = Math.hypot(targetX - startX, targetY - startY);

        // Compute acceleration phase distance
        timeToMaxVelocity = maxVelocity / maxAcceleration;
        double distanceDuringAccel = 0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity;

        // Determine if a triangular or trapezoidal profile is needed.
        if (2 * distanceDuringAccel > totalDistance) {
            // Triangular profile: cannot reach max velocity
            timeToMaxVelocity = Math.sqrt(totalDistance / maxAcceleration);
            peakVelocity = maxAcceleration * timeToMaxVelocity;
            cruiseTime = 0;
            totalTime = 2 * timeToMaxVelocity;
        } else {
            // Trapezoidal profile
            peakVelocity = maxVelocity;
            cruiseTime = (totalDistance - 2 * distanceDuringAccel) / maxVelocity;
            totalTime = 2 * timeToMaxVelocity + cruiseTime;
        }

        // Initialize timer and dynamic fields.
        startTime = System.currentTimeMillis();
        timeElapsed = 0;
        phase = 0;
    }

    /**
     * Computes the instantaneous target position along the straight-line path.
     *
     * @return a double array with {currentX, currentY}.
     */
    public double[] getTargetPosition() {
        timeElapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        double distanceTraveled = 0;

        if (timeElapsed < timeToMaxVelocity) {
            // Accelerating phase: s = 0.5 * a * t^2
            distanceTraveled = 0.5 * maxAcceleration * timeElapsed * timeElapsed;
            phase = 0;
        } else if (timeElapsed < timeToMaxVelocity + cruiseTime) {
            // Cruising phase
            distanceTraveled = (0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity)
                    + peakVelocity * (timeElapsed - timeToMaxVelocity);
            phase = 1;
        } else if (timeElapsed < totalTime) {
            // Decelerating phase
            double tDecel = timeElapsed - timeToMaxVelocity - cruiseTime;
            distanceTraveled = (0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity)
                    + peakVelocity * cruiseTime
                    + peakVelocity * tDecel - 0.5 * maxAcceleration * tDecel * tDecel;
            phase = 2;
        } else {
            // Profile complete
            distanceTraveled = totalDistance;
            phase = 3;
        }

        // Compute the ratio along the path and clamp to 1
        double ratio = (totalDistance == 0) ? 1 : (distanceTraveled / totalDistance);
        ratio = Math.min(ratio, 1);

        double currentX = startX + (targetX - startX) * ratio;
        double currentY = startY + (targetY - startY) * ratio;

        return new double[]{currentX, currentY};
    }

    /**
     * Returns true if the motion profile has completed.
     */
    public boolean profileComplete() {
        timeElapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        return timeElapsed >= totalTime+0.05;
    }

    /**
     * Returns the instantaneous target state as an array:
     * {targetX, targetY, targetVelocity, targetAcceleration}.
     */
    public double[] getTargetState() {
        timeElapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        double tv, ta;
        double a = maxAcceleration;
        if (timeElapsed < timeToMaxVelocity) {
            tv = a * timeElapsed;
            ta = a;
            phase = 0;
        } else if (timeElapsed < timeToMaxVelocity + cruiseTime) {
            tv = peakVelocity;
            ta = 0;
            phase = 1;
        } else if (timeElapsed < totalTime) {
            double tDecel = timeElapsed - timeToMaxVelocity - cruiseTime;
            tv = peakVelocity - a * tDecel;
            ta = -a;
            phase = 2;
        } else {
            tv = 0;
            ta = 0;
            phase = 3;
        }
        double[] pos = getTargetPosition();
        return new double[]{ pos[0], pos[1], tv, ta };
    }
}
