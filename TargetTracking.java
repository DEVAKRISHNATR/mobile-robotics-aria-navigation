package exercises.e3;

import org.jfree.data.xy.XYSeries;
import plot.LinePlotter;
import robot.Robot;

public class TargetTracking {
    private double x;
    private double p = 1; // Estimate uncertainty
    private double q = 0.1; // Process noise
    private double r = 1; // Measurement noise
    private double k;

    private LinePlotter plotter;
    private XYSeries series[] = {
            new XYSeries("Measurement"),
            new XYSeries("Estimate")
    };

    private Robot robot;

    public TargetTracking(Robot robot) {
        this.robot = robot;
        plotter = new LinePlotter(series);
        plotter.setVisible(true);
        x = robot.sensor.getBlobX() / 2;
    }

    public boolean zoneTracker(double vel) {
        // STOP if no blob is detected
        if (!isBlobDetected()) {
            robot.arRobot.setVel(0);
            robot.arRobot.setRotVel(0);
            System.out.println("No blob detected - Stopping.");
            return false;
        }

        double blobX = robot.sensor.getBlobX();
        double imageWidth = robot.sensor.getImageWidth();

        System.out.println("Blob Detected at X: " + blobX + " / Image Width: " + imageWidth);

        if (blobX < imageWidth / 3) {
            robot.arRobot.setRotVel(-vel); // Turn left
            robot.arRobot.setVel(0); // Stop moving forward
        } else if (blobX > 2 * imageWidth / 3) {
            robot.arRobot.setRotVel(vel); // Turn right
            robot.arRobot.setVel(0); // Stop moving forward
        } else {
            robot.arRobot.setRotVel(0); // Stop rotating
            robot.arRobot.setVel(vel); // Move forward
        }
        return true;
    }

    public boolean kalmanTracker(double vel) {
        // STOP if no blob is detected
        if (!isBlobDetected()) {
            robot.arRobot.setVel(0);
            robot.arRobot.setRotVel(0);
            System.out.println("No blob detected - Stopping.");
            return false;
        }

        double measurement = robot.sensor.getBlobX();
        x = getKalmanEstimate(measurement);
        double mappedVel = map(x, 0, robot.sensor.getImageWidth(), -vel, vel);

        System.out.println("Blob Detected. Mapped Velocity: " + mappedVel);

        // Only move forward if the blob is centered
        if (Math.abs(mappedVel) < vel / 4) {
            robot.arRobot.setRotVel(0);
            robot.arRobot.setVel(vel);
        } else {
            robot.arRobot.setRotVel(mappedVel);
            robot.arRobot.setVel(0);
        }

        return true;
    }

    public double getKalmanEstimate(double measurement) {
        p += q;
        k = p / (p + r);
        x += k * (measurement - x);
        p *= (1 - k);
        plotter(measurement, x);
        return x;
    }

    public boolean approach(double vel) {
        // STOP if no blob is detected
        if (!isBlobDetected()) {
            robot.arRobot.setVel(0);
            System.out.println("No blob detected - Stopping.");
            return false;
        }

        double distance = getSensorDistance();
        if (distance > 500) {
            robot.arRobot.setVel(vel);
        } else {
            robot.arRobot.setVel(0);
        }
        return true;
    }

    private double getSensorDistance() {
        System.out.println("Warning: Using placeholder getSensorDistance()");
        return 1000; // Default distance for testing
    }

    public double map(double x, double setA1, double setA2, double setB1, double setB2) {
        return (x - setA1) / (setA2 - setA1) * (setB2 - setB1) + setB1;
    }

    public void plotter(double m, double e) {
        long ms = System.currentTimeMillis();
        series[0].add(ms, m);
        series[1].add(ms, e);
        System.out.println("Measurement: " + m + ", Estimate: " + e);
    }

    private boolean isBlobDetected() {
        double blobX = robot.sensor.getBlobX();
        return blobX > 0; // Assuming valid blobs have X > 0
    }
}