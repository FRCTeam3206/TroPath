package frc.utils;

import java.util.function.Predicate;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandDuringPath {
    private Command command;
    private Predicate<Double> isActivated;

    /**
     * This should not be used by the user.
     */
    public CommandDuringPath(Command command, double startDistance, double endDistance) {
        this.command = command;

        // Uses inputting negatives to be told to not do start or end distance.
        boolean hasStart = startDistance > 0;
        boolean hasEnd = endDistance > 0;
        if (hasStart && hasEnd) {
            if (startDistance > endDistance) {
                isActivated = (Double dist) -> dist < startDistance && dist > endDistance;
            } else {
                isActivated = (Double dist) -> dist < endDistance && dist > startDistance;
            }
        } else if (hasStart) {
            isActivated = (Double dist) -> dist < startDistance;
        } else if (hasEnd) {
            isActivated = (Double dist) -> dist > endDistance;
        } else {
            isActivated = (Double dist) -> true;
        }
    }

    public CommandDuringPath(Command command, Predicate<Double> isActivatedGivenDist) {
        this.command = command;
        isActivated = isActivatedGivenDist;
    }

    public Command getCommand() {
        return command;
    }

    public boolean getIsActivated(double dist) {
        return isActivated.test(dist);
    }
}