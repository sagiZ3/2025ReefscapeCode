package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CustomLEDPatterns;

import java.util.function.Supplier;

import static frc.lib.util.CustomLEDPatterns.LEDS_COUNT;
import static frc.lib.util.CustomLEDPatterns.generateBreathingBuffer;
import static frc.lib.util.CustomLEDPatterns.generateCirclingBuffer;
import static frc.lib.util.CustomLEDPatterns.generateFlashingBuffer;
import static frc.lib.util.CustomLEDPatterns.generateOutwardsPointsBuffer;
import static frc.lib.util.CustomLEDPatterns.getBufferFromColours;

public class Leds extends SubsystemBase {
    private static final AddressableLED ledstrip = new AddressableLED(0);
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_COUNT);

    public Leds() {
        ledstrip.setLength(LEDS_COUNT);
        ledstrip.setData(buffer);
        ledstrip.start();
    }

    public static void correctRobotPositionFade(Translation2d robotPose, Translation2d targetPose) {
        final double deltaX = robotPose.getX() - targetPose.getX();
        final double deltaY = robotPose.getY() - targetPose.getY();

        final double maxDistance = 2.0;

        final double normalizedY = Math.min(Math.abs(deltaY) / maxDistance, 1.0);
        final double normalizedX = Math.min(Math.abs(deltaX) / maxDistance, 1.0);

        Color leftColor = Color.kBlack;
        Color rightColor = Color.kBlack;
        Color frontColor = Color.kBlack;
        Color backColor = Color.kBlack;

        if (deltaX > 0) leftColor = interpolateColor(normalizedX);

        if (deltaX < 0) rightColor = interpolateColor(normalizedX);

        if (deltaY > 0) backColor = interpolateColor(normalizedY);

        if (deltaY < 0) frontColor = interpolateColor(normalizedY);

        buffer.setLED(0, leftColor);
        buffer.setLED(46, leftColor);

        buffer.setLED(23, rightColor);
        buffer.setLED(24, rightColor);

        buffer.setLED(11, frontColor);
        buffer.setLED(12, frontColor);

        buffer.setLED(34, backColor);
        buffer.setLED(35, backColor);
    }

    private static Color interpolateColor(double distance) {
        final int r = (int) (255 * distance);
        final int g = (int) (255 * (1 - distance));

        return new Color(r, g, 0);
    }


    public Command setLEDStatus(LEDMode mode, double timeout) {
        return switch (mode) {
            case SHOOTER_LOADED -> getCommandFromColours(() -> generateFlashingBuffer(
                    new Color8Bit(Color.kOrange),
                    new Color8Bit(Color.kRed)
            ), timeout);

            case SHOOTER_EMPTY -> getCommandFromColours(() -> generateCirclingBuffer(
                    new Color8Bit(Color.kCyan),
                    new Color8Bit(Color.kWhite),
                    new Color8Bit(Color.kDeepPink)
            ), timeout);

            case DEBUG_MODE -> getCommandFromColours(() -> generateBreathingBuffer(
                    new Color8Bit(Color.kCyan),
                    new Color8Bit(Color.kWhite)
            ), timeout);

            case NOT_AT_AUTO_PLACE -> getCommandFromColours(() -> generateBreathingBuffer(
                    new Color8Bit(Color.kRed),
                    new Color8Bit(Color.kGreen)
            ), timeout);

            case BATTERY_LOW -> getCommandFromColours(() -> generateOutwardsPointsBuffer(new Color8Bit(Color.kRed)), timeout);

            default -> getCommandFromColours(CustomLEDPatterns::generateRainbowBuffer, 0);
        };
    }

    private Command getCommandFromColours(Supplier<Color8Bit[]> colours, double timeout) {
        if (timeout == 0)
            return Commands.run(() -> flashLEDStrip(colours.get()), this).ignoringDisable(true);

        return Commands.run(
                () -> flashLEDStrip(colours.get()), this).withTimeout(timeout).ignoringDisable(true);
    }

    private void flashLEDStrip(Color8Bit[] colours) {
        ledstrip.setData(getBufferFromColours(buffer, colours));
    }

    public enum LEDMode {
        SHOOTER_LOADED,
        SHOOTER_EMPTY,
        DEBUG_MODE,
        BATTERY_LOW,
        DEFAULT,
        NOT_AT_AUTO_PLACE
    }
}
