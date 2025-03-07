package frc.robot;

import java.util.ArrayList;

public class MovingAverage {
    int window;
    double currentAverage = 0;
    double sum = 0;
    ArrayList<Double> data = new ArrayList<Double>();
    public MovingAverage(int window) {
        this.window = window;
    }
    public void putData(double value) {
        data.add(value);
        sum += value;
        if (data.size() > window) {
            sum -= data.get(0);
            data.remove(0);
        }
        currentAverage = sum / data.size();
    }
    public void clear() {
        
    }
}