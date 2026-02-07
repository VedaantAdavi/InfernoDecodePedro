package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class InterpolatedLUT {
    // Simple bubble sort to keep keys and values in sync
    private static void sort(ArrayList<Double> keys, ArrayList<Double> values) {
        for (int i = 0; i < keys.size() - 1; i++) {
            for (int j = 0; j < keys.size() - i - 1; j++) {
                if (keys.get(j) > keys.get(j + 1)) {
                    double tempKey = keys.get(j);
                    double tempValue = values.get(j);
                    keys.set(j, keys.get(j + 1));
                    values.set(j, values.get(j + 1));
                    keys.set(j + 1, tempKey);
                    values.set(j + 1, tempValue);
                }
            }
        }
    }

    // Get interpolated value
    public static double get(ArrayList<Double> keys, ArrayList<Double> values, double key) {
        sort(keys, values);

        if (keys.isEmpty()) {
            throw new IllegalStateException("LUT is empty");
        }

        // Exact match
        if (keys.contains(key)) {
            return values.get(keys.indexOf(key));
        }

        // Linear interpolation
        for (int i = 0; i < keys.size() - 1; i++) {
            if (key > keys.get(i) && key < keys.get(i + 1)) {
                double slope = (values.get(i + 1) - values.get(i)) / (keys.get(i + 1) - keys.get(i));
                return values.get(i) + slope * (key - keys.get(i));
            }
        }

        // Extrapolation (clamping)
        if (key < keys.get(0)) {
            return values.get(0);
        }
        return values.get(values.size() - 1);
    }
}
