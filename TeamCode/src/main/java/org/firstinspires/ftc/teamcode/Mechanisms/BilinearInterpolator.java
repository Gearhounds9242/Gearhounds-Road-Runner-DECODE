package org.firstinspires.ftc.teamcode.Mechanisms;

import java.util.TreeMap;
import java.util.Map;

/**
 * BilinearTable — general-purpose 2D lookup table with bilinear interpolation.
 *
 * Usage:
 *   BilinearTable table = new BilinearTable()
 *       .add(0, 0, 0.0)
 *       .add(0, 10, 0.5)
 *       .add(10, 0, 0.3)
 *       .add(10, 10, 1.0);
 *
 *   double result = table.get(5, 5); // → 0.45
 *
 * - X and Y axes are automatically built from added points.
 * - Queries outside the data range are clamped to the nearest edge.
 * - Points do NOT need to be added in order.
 */
public class BilinearInterpolator {

    // Outer key = X, inner key = Y, value = Z (output)
    private final TreeMap<Double, TreeMap<Double, Double>> data = new TreeMap<>();

    /**
     * Add a data point to the table.
     *
     * @param x  first axis value (e.g. distance, speed, angle)
     * @param y  second axis value
     * @param   output value at (x, y)
     * @return   this table, for chaining
     */
    public BilinearInterpolator add(double x, double y, double output) {
        data.computeIfAbsent(x, k -> new TreeMap<>()).put(y, output);
        return this;
    }

    /**
     * Look up an interpolated value at (x, y).
     * Out-of-range inputs are clamped to the nearest known value.
     *
     * @param x  query X
     * @param y  query Y
     * @return   interpolated (or clamped) output value
     */
    public double get(double x, double y) {
        if (data.isEmpty()) throw new IllegalStateException("Table has no data points.");

        // Clamp X to known range
        double x0 = clampKey(data, x);
        Double x1Raw = data.higherKey(x);
        double x1 = (x1Raw == null) ? x0 : x1Raw;

        if (x0 == x1) {
            // Exact X match or clamped edge — just interpolate along Y
            return interpY(data.get(x0), y);
        }

        double zAtX0 = interpY(data.get(x0), y);
        double zAtX1 = interpY(data.get(x1), y);

        double tx = (x - x0) / (x1 - x0);
        return lerp(zAtX0, zAtX1, tx);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    /** Interpolate along Y within a single row (fixed X). */
    private double interpY(TreeMap<Double, Double> row, double y) {
        if (row == null) throw new IllegalStateException("Missing row in table.");

        double y0 = clampKey(row, y);
        Double y1Raw = row.higherKey(y);
        double y1 = (y1Raw == null) ? y0 : y1Raw;

        if (y0 == y1) return row.get(y0);

        double ty = (y - y0) / (y1 - y0);
        return lerp(row.get(y0), row.get(y1), ty);
    }

    /** Return the closest key in a TreeMap, clamped to [min, max]. */
    private <V> double clampKey(TreeMap<Double, V> map, double query) {
        Map.Entry<Double, V> floor = map.floorEntry(query);
        if (floor != null) return floor.getKey();
        return map.firstKey(); // query is below range — clamp to min
    }

    /** Standard linear interpolation: a + t*(b-a), t in [0,1]. */
    private double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

//    // ── Optional: debug dump ─────────────────────────────────────────────────
//
//    /** Print the table to stdout — handy for tuning on the robot. */
//    public void print() {
//        System.out.println("BilinearTable contents:");
//        for (Map.Entry<Double, TreeMap<Double, Double>> xEntry : data.entrySet()) {
//            for (Map.Entry<Double, Double> yEntry : xEntry.getValue().entrySet()) {
//                System.out.printf("  x=%-8.2f y=%-8.2f → %.4f%n",
//                        xEntry.getKey(), yEntry.getKey(), yEntry.getValue());
//            }
//        }
//    }
}