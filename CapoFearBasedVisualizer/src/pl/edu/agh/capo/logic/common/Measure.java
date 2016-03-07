package pl.edu.agh.capo.logic.common;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public class Measure {
    private final double leftVelocity;
    private final double rightVelocity;
    private final List<Vision> visions;

    public Measure(double leftVelocity, double rightVelocity, List<Vision> visions) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.visions = visions;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public List<Vision> getVisions() {
        return visions;
    }
}
