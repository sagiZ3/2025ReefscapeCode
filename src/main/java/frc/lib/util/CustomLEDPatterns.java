package frc.lib.util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;

public class CustomLEDPatterns {
    public static final int LEDS_COUNT = 46;
    protected final static double MAX_GREEN_RANGE = 2;

    private static int counter;
    private static int previousColour = 0;

    private static final Timer timer = new Timer();

    private static Color8Bit[] buffer = new Color8Bit[LEDS_COUNT];

    private static int rainbowFirstPixel;

    static {
        timer.start();
    }

    public static Color8Bit[] reduceBrightness(Color8Bit[] colours, int brightnessPercentage) {
        double brightnessFactor = brightnessPercentage / 100.0;

        final Color8Bit[] adjustedColours = new Color8Bit[colours.length];

        for (int i = 0; i < colours.length; i++) {
            Color8Bit originalColor = colours[i];

            int newRed = (int) (originalColor.red * brightnessFactor);
            int newGreen = (int) (originalColor.green * brightnessFactor);
            int newBlue = (int) (originalColor.blue * brightnessFactor);

            newRed = Math.min(255, Math.max(0, newRed));
            newGreen = Math.min(255, Math.max(0, newGreen));
            newBlue = Math.min(255, Math.max(0, newBlue));

            adjustedColours[i] = new Color8Bit(newRed, newGreen, newBlue);
        }

        return adjustedColours;
    }

    /**
     * Generates a buffer that indicates how far in each direction (left, right, forwards, backwards)
     * the robot is from the correct position. This should be called periodically to update the indicator.
     * If a certain direction is perfect, it turns off that direction's LEDs.
     *
     * @param startColor The color when the robot is at the furthest position.
     * @param endColor   The color when the robot is at the closest position.
     * @param robotPose  The current robot position.
     * @param targetPose The target position.
     * @return The filled buffer with corrective LED colors.
     */
    public static Color8Bit[] generatePositionIndicatorBuffer(Color8Bit startColor, Color8Bit endColor, Translation2d robotPose, Translation2d targetPose) {
        final double deltaX = robotPose.getX() - targetPose.getX();
        final double deltaY = robotPose.getY() - targetPose.getY();

        final double normalizedY = Math.min(Math.abs(deltaY) / MAX_GREEN_RANGE, 1.0);
        final double normalizedX = Math.min(Math.abs(deltaX) / MAX_GREEN_RANGE, 1.0);

        Color8Bit leftColor = new Color8Bit(Color.kBlack);
        Color8Bit rightColor = new Color8Bit(Color.kBlack);
        Color8Bit frontColor = new Color8Bit(Color.kBlack);
        Color8Bit backColor = new Color8Bit(Color.kBlack);

        if (deltaX > 0) leftColor = interpolateColors(startColor, endColor, normalizedX);

        if (deltaX < 0) rightColor = interpolateColors(startColor, endColor, normalizedX);

        if (deltaY > 0) backColor = interpolateColors(startColor, endColor, normalizedY);

        if (deltaY < 0) frontColor = interpolateColors(startColor, endColor, normalizedY);

        buffer[0] = leftColor;
        buffer[46] = leftColor;
        buffer[23] = rightColor;
        buffer[24] = rightColor;
        buffer[11] = frontColor;
        buffer[12] = frontColor;
        buffer[34] = backColor;
        buffer[35] = backColor;

        return buffer;
    }

    /**
     * Generates a buffer with a single color.
     *
     * @param color The color to fill the buffer.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateSingleColourBuffer(Color8Bit colour) {
        Arrays.fill(buffer, colour);
        return buffer;
    }

    /**
     * Set the buffer from the color.
     *
     * @param ledBuffer The LED buffer to set.
     * @param buffer The color buffer.
     * @return The updated LED buffer.
     */
    public static AddressableLEDBuffer getBufferFromColours(AddressableLEDBuffer ledBuffer, Color8Bit[] buffer) {
        for (int i = 0; i < buffer.length; i++) {
            ledBuffer.setLED(i, buffer[i]);
        }

        return ledBuffer;
    }

    /**
     * Fill the buffer with RAINBOW colors. This needs to be called periodically for the rainbow effect
     * to be dynamic.
     *
     * @return The filled buffer.
     */
    public static Color8Bit[] generateRainbowBuffer() {
        int hue;

        for (int i = 0; i < LEDS_COUNT; i++) {
            hue = (rainbowFirstPixel + (i * 180 / LEDS_COUNT)) % 180;

            buffer[i] = new Color8Bit(Color.fromHSV(hue, 255, 128));
        }

        rainbowFirstPixel += 3;
        rainbowFirstPixel %= 180;

        return buffer;
    }

    /**
     * Set the buffer to flash between a set of colors. This needs to be called periodically for the
     * flashing effect to work.
     *
     * @param colors The colors to switch between.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateFlashingBuffer(Color8Bit... colours) {
        if (counter % 25 == 0) //Make sure there's a delay between colour switching
            buffer = generateSingleColourBuffer(colours[previousColour++]);

        previousColour %= colours.length;
        counter++;

        return buffer;
    }

    /**
     * Generates a loading animation that moves outwards from the center of the LED strip.
     * This should be called periodically to update the animation.
     *
     * @param color1 The color for the first direction.
     * @param color2 The color for the second direction.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateLoadingAnimationBuffer(Color8Bit color1, Color8Bit color2) {
        buffer = generateSingleColourBuffer(new Color8Bit(Color.kBlack)); // Clear the buffer

        int midPoint = LEDS_COUNT / 2; // Find the middle of the LED strip
        double time = timer.get() * 5; // Control the speed of the animation
        int progress = (int) time % (LEDS_COUNT / 2); // Get the current progress

        for (int i = midPoint - progress; i <= midPoint; i++) {
            if (i >= 0) {
                buffer[i] = color1;
            }
        }

        for (int i = midPoint + progress; i >= midPoint; i--) {
            if (i < LEDS_COUNT) {
                buffer[i] = color2;
            }
        }

        return buffer;
    }

    /**
     * Slowly switches between two colors, creating a breathing effect.
     *
     * @param firstColor  The first color.
     * @param secondColor The second color.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateBreathingBuffer(Color8Bit firstColour, Color8Bit secondColour) {
        double x = timer.get();
        return generateSingleColourBuffer(interpolateColours(firstColour, secondColour, cosInterpolate(x)));
    }

    /**
     * Circles through N colors across the LED strip, utilizing a smooth effect.
     * This should be called periodically.
     *
     * @param colors The colors to cycle through.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateCirclingBuffer(Color8Bit... colours) {
        int colorsLength = colours.length;
        float timerValue = (float) timer.get(); // Get current timer value
        float timerPosition = timerValue * 13f % LEDS_COUNT; // Adjust the multiplier to control the speed
        final double timerValue = timer.get();
        final double timerPosition = timerValue * 13 % LEDS_COUNT;

        int index, colorIndex1, colorIndex2;
        double colorIndex, fraction;

        for (int i = 0; i < LEDS_COUNT; i++) {
            index = wrapIndex(i);

            colorIndex = (timerPosition + i) % LEDS_COUNT / LEDS_COUNT * colorsLength;

            colorIndex1 = (int) colorIndex;
            colorIndex2 = (colorIndex1 + 1) % colorsLength;

            fraction = cosInterpolate(colorIndex - colorIndex1);

            Color8Bit color1 = colors[colorIndex1];
            Color8Bit color2 = colors[colorIndex2];

            buffer[index] = interpolateColors(color1, color2, fraction);
        }

        return buffer;
    }

    /**
     * Clears the buffer, then moves a color from the middle outwards.
     * Creating a nice loading effect. Should be used periodically.
     *
     * @param color The color to use
     * @return The current state of the buffer
     */
    public static Color8Bit[] generateOutwardsPointsBuffer(Color8Bit colour) {
        buffer = generateSingleColourBuffer(new Color8Bit(Color.kBlack));

        final int quarter = LEDS_COUNT / 4;

        final double time = timer.get();

        int x = time == (int) time ? ((int) (time) % 11) : ((int) (time * 16 % 11));

        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
            buffer[i] = new Color8Bit(new Color(colour.red, colour.green, colour.blue));
        }

        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
            buffer[i] = new Color8Bit(new Color(colour.red, colour.green, colour.blue));
        }

        return buffer;
    }

    private static int wrapIndex(int i) {
        while (i >= LEDS_COUNT)
            i -= LEDS_COUNT;

        while (i < 0) {
            i += LEDS_COUNT;
        }

        return i;
    }


    private static Color8Bit interpolateColors(Color8Bit color1, Color8Bit color2, double colorWeight) {
        int red = (int) (color1.red * (1 - colorWeight) + color2.red * colorWeight);
        int green = (int) (color1.green * (1 - colorWeight) + color2.green * colorWeight);
        int blue = (int) (color1.blue * (1 - colorWeight) + color2.blue * colorWeight);

        return new Color8Bit(red, green, blue);
    }

    private static double cosInterpolate(double x) {
        return ((1 - Math.cos(x * Math.PI)) * 0.5);
    }
}
