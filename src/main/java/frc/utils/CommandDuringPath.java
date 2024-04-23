package frc.utils;

import java.util.function.Predicate;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandDuringPath {
    private Command command;
    private Predicate<Double> isActivate;

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
                isActivate = (Double dist) -> dist < startDistance && dist > endDistance;
            } else {
                isActivate = (Double dist) -> dist < endDistance && dist > startDistance;
            }
        } else if (hasStart) {
            isActivate = (Double dist) -> dist < startDistance;
        } else if (hasEnd) {
            isActivate = (Double dist) -> dist > endDistance;
        } else {
            isActivate = (Double dist) -> true;
        }
    }

    public CommandDuringPath(Command command, Predicate<Double> isActivateGivenDist) {
        this.command = command;
        isActivate = isActivateGivenDist;
    }

    public Command getCommand() {
        return command;
    }

    public boolean getIsActive(double dist) {
        return isActivate.test(dist);
    }
}