package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Encoder;

public class ElevatorEncoderBoolean implements BooleanSupplier, Supplier<Boolean>{

    private final int height;
    private final Encoder encoder;
    private final boolean up;

    public ElevatorEncoderBoolean(int height, Encoder encoder, boolean up) {
        this.height = height;
        this.encoder = encoder;
        this.up = up;
    }

    @Override
    public Boolean get() {
        return getAsBoolean();
    }

    @Override
    public boolean getAsBoolean() {
        if (up) {
            return (encoder.get() >= height);
        } else {
            return (encoder.get() < height);
        }
    }
}
