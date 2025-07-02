package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;
import java.util.function.Function;

public class Pipeline<T> {

    protected ArrayList<Function<T, T>> steps;

    public Pipeline() {

        this.steps = new ArrayList<>();

    }

    public Pipeline<T> addStep(Function<T, T> step) {

        this.steps.add(step);
        return this;

    }

    public T apply(T input) {

        return this.steps.stream().reduce(
            input,
            (current, step) -> step.apply(current),
            (a, b) -> a // This is not used in a sequential pipeline
        );

    }

}
