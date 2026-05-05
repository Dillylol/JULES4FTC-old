package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * Shooter Configuration - Tunable via Panels UI
 */
@Configurable
public class ShooterConfigurables {

    @Sorter(sort = 0)
    public static double voltageAlpha = 0.1; // Smoothing factor for voltage

    @Sorter(sort = 1)
    public static double rpmEmaAlpha = 0.2;

    @Sorter(sort = 2)
    public static double readyTolRpm = 100.0;

    @Sorter(sort = 3)
    public static int idleRpm = 2000;

    @Sorter(sort = 4)
    public static double nominalBattV = 12.0;

    @Sorter(sort = 5)
    public static double shooterKVRpm = 0.0; // RPM added per volt of sag

    @Sorter(sort = 6)
    public static int maxRpmStepPerUpdate = 250;

    @Sorter(sort = 7)
    public static int rampDurationMs = 3000;

    @Sorter(sort = 8)
    public static double rampCoef = 0.00025;

    @Sorter(sort = 9)
    public static double rampMinRate = 400.0;

    @Sorter(sort = 10)
    public static double rpmSlopeRanger = 116.4042383594456;

    @Sorter(sort = 11)
    public static double rpmOffsetRanger = 2084.2966941424975;

    @Sorter(sort = 12)
    public static double minRpm = 1000.0;

    @Sorter(sort = 13)
    public static double maxRpm = 6000.0;
}
