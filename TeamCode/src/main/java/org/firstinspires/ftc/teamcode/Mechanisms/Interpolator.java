package org.firstinspires.ftc.teamcode.Mechanisms;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Interpolator {

    private final List<DataPoint> points = new ArrayList<>();
    private final int neighborCount;
    private final double power;

    public Interpolator(int neighborCount, double power) {
        this.neighborCount = neighborCount;
        this.power = power;
    }

    public void addPoint(double x, double y, double value) {
        points.add(new DataPoint(x, y, value));
    }

    public void clear() {
        points.clear();
    }

    public double interpolate(double x, double y) {

        if (points.isEmpty()) {
            throw new IllegalStateException("No data points available.");
        }

        // Sort by distance (could be replaced by spatial tree later)
        List<DataPoint> sorted = new ArrayList<>(points);
        sorted.sort(Comparator.comparingDouble(p -> distanceSq(p, x, y)));

        int count = Math.min(neighborCount, sorted.size());

        double numerator = 0.0;
        double denominator = 0.0;

        for (int i = 0; i < count; i++) {
            DataPoint p = sorted.get(i);

            double distSq = distanceSq(p, x, y);

            if (distSq == 0) {
                return p.value;
            }

            double weight = 1.0 / Math.pow(distSq, power / 2.0);
            numerator += weight * p.value;
            denominator += weight;
        }

        return numerator / denominator;
    }

    private double distanceSq(DataPoint p, double x, double y) {
        double dx = x - p.x;
        double dy = y - p.y;
        return dx * dx + dy * dy;
    }
}




//Interpolator interpolator = new Interpolator(8, 2.0);
//
//        interpolator.addPoint(0, 0, 10);
//        interpolator.addPoint(10, 0, 20);
//        interpolator.addPoint(0, 10, 30);
//        interpolator.addPoint(10, 10, 40);
//        interpolator.addPoint(5, 5, 25);
//
//double result = interpolator.interpolate(4, 6);