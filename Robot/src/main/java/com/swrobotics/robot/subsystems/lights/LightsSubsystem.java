package com.swrobotics.robot.subsystems.lights;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class LightsSubsystem extends SubsystemBase {
    private final RobotContainer robot;
    private final AddressableLED leds;
    private final AddressableLEDBuffer data;

    private final Debouncer batteryLowDebounce;
    private final PrideSequencer prideSequencer;

    private Color commandRequest = null;

    public LightsSubsystem(RobotContainer robot) {
        this.robot = robot;
        leds = new AddressableLED(IOAllocation.RIO.PWM_LEDS);
        leds.setLength(Constants.kLedStripLength);

        data = new AddressableLEDBuffer(Constants.kLedStripLength);
        leds.setData(data);

        leds.start();

        batteryLowDebounce = new Debouncer(10);
        prideSequencer = new PrideSequencer();
    }

    private void showOverheating() {
        applySolid(Timer.getFPGATimestamp() % 0.4 > 0.2 ? Color.kRed : Color.kBlack);
    }

    private void showLowBattery() {
        applySolid(Timer.getFPGATimestamp() % 0.4 > 0.2 ? Color.kOrange : Color.kBlack);
    }

    private void showAutoDriving() {
        // Rainbow
        applyStripes(5f,
                new Stripe(Color.kRed, 1),
                new Stripe(Color.kOrange, 1),
                new Stripe(Color.kYellow, 1),
                new Stripe(Color.kGreen, 1),
                new Stripe(Color.kBlue, 1),
                new Stripe(Color.kPurple, 1));
    }

    private void showIdle() {
        applySolid(Color.kBlack);
    }

    @Override
    public void periodic() {
        boolean overCurrent = robot.pdp.getInputs().pdpTotalCurrent > Constants.kLedCurrentShutoffThreshold;
        boolean overheating = robot.temperatureTracker.isOverheating();
        boolean batteryLow = RobotController.getBatteryVoltage() < Constants.kLowBatteryThreshold;

        if (overCurrent) {
            applySolid(Color.kBlack); // Black means LEDs off
        } else if (overheating) {
            showOverheating();
        } else if (batteryLowDebounce.calculate(batteryLow)) {
            showLowBattery();
        } else if (commandRequest != null) {
            applySolid(commandRequest);
        } else if (DriverStation.isDisabled()) {
            prideSequencer.apply(this);
        } else if (robot.drive.getLastSelectedPriority() == SwerveDriveSubsystem.Priority.AUTO) {
            showAutoDriving();
        } else {
            showIdle();
        }

        commandRequest = null;
    }

    private void applySolid(Color color) {
        for (int i = 0; i < Constants.kLedStripLength; i++) {
            data.setLED(i, color);
        }
        leds.setData(data);
    }

    public static final record Stripe(Color color, float weight) {}

    private Color interpolate(Color a, Color b, float percent) {
        return new Color(
                MathUtil.lerp(a.red, b.red, percent),
                MathUtil.lerp(a.green, b.green, percent),
                MathUtil.lerp(a.blue, b.blue, percent)
        );
    }

    private static final record StripeBoundary(float position, Stripe toRight) {}

    // Scroll speed is seconds per full pass through the pattern
    // TODO: optimize this
    public void applyStripes(float scrollSpeed, Stripe... stripes) {
        float totalWeight = 0;
        for (Stripe stripe : stripes)
            totalWeight += stripe.weight;

        float scroll = scrollSpeed == 0 ? 0 : (float) Timer.getFPGATimestamp() / scrollSpeed;

        // Pattern is sampled at pixel's left edge
        List<StripeBoundary> boundaries = new ArrayList<>();
        float weightSoFar = 0;
        for (Stripe stripe : stripes) {
            float leftEdgePos = (float) MathUtil.floorMod(weightSoFar / totalWeight + scroll, 1) * Constants.kLedStripLength;
            boundaries.add(new StripeBoundary(leftEdgePos, stripe));
            weightSoFar += stripe.weight;
        }
        boundaries.sort(Comparator.comparingDouble(StripeBoundary::position));

        for (int pixel = 0; pixel < Constants.kLedStripLength; pixel++) {
            StripeBoundary before = null, after = null;
            for (StripeBoundary boundary : boundaries) {
                if (boundary.position > pixel) {
                    after = boundary;
                    break;
                }
                before = boundary;
            }

            if (before == null)
                before = boundaries.get(boundaries.size() - 1);
            if (after == null) {
                data.setLED(pixel, before.toRight.color);
                continue;
            }

            if (pixel == (int) after.position) {
                float percent = 1 - (after.position % 1);
                data.setLED(pixel, interpolate(before.toRight.color, after.toRight.color, percent));
                continue;
            }

            data.setLED(pixel, before.toRight.color);
        }

        leds.setData(data);
    }

    public void disabledInit() {
        prideSequencer.reset();
    }

    public void setCommandRequest(Color color) {
        commandRequest = color;
    }
}
