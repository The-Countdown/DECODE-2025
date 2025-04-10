package org.firstinspires.ftc.teamcode.util;

public class MathEx {

    /**
     * Calculates the sum of an arithmetic series.
     *
     * @param n  The number of terms in the series.
     * @param a1 The first term of the series.
     * @param an The last term of the series.
     * @return The sum of the arithmetic series.
     * @throws IllegalArgumentException if n is not positive
     */
    public static double arithmeticSeriesSum(int n, double a1, double an) {
        if (n <= 0) {
            throw new IllegalArgumentException("The number of terms (n) must be positive.");
        }
        return (n / 2.0) * (a1 + an);
    }

    /**
     * alternative implementation for a sigma sum
     * @param n the end of the sum
     * @param a1 the start of the sum
     * @param d the increment
     * @return the sum from a1 to n with an increment of d
     */
    public static double sigmaSum(int n, double a1, double d) {
        if (n <= 0) {
            throw new IllegalArgumentException("The number of terms (n) must be positive.");
        }
        double an = a1 + (n - 1) * d;
        return (n / 2.0) * (a1 + an);
    }
}