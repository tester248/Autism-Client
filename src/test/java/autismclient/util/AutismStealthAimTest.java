package autismclient.util;

import autismclient.util.AutismRotationUtil.Rotation;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class AutismStealthAimTest {

    private static AutismStealthAim.Config quietConfig() {
        AutismStealthAim.Config config = new AutismStealthAim.Config();
        config.wanderMaxDeg = 0.0D;
        config.pauses = false;
        config.lag = false;
        config.overshoot = false;
        return config;
    }

    private static double wrapD(double degrees) {
        double wrapped = degrees % 360.0D;
        if (wrapped >= 180.0D) wrapped -= 360.0D;
        if (wrapped < -180.0D) wrapped += 360.0D;
        return wrapped;
    }

    @Test
    void disabledConfigReturnsGoalIdentity() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = quietConfig();
        config.enabled = false;
        Random random = new Random(1L);
        Rotation goal = new Rotation(42.0F, -7.0F);
        Rotation current = new Rotation(10.0F, 3.0F);
        for (int i = 0; i < 50; i++) {
            Rotation out = AutismStealthAim.filter(state, config, random, goal, current);
            assertEquals(goal.yaw(), out.yaw(), 1.0E-6F);
            assertEquals(goal.pitch(), out.pitch(), 1.0E-6F);
            current = out;
        }
    }

    @Test
    void wanderStaysBoundedWhileSettled() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = quietConfig();
        config.wanderMaxDeg = 0.4D;
        Random random = new Random(7L);
        Rotation goal = new Rotation(30.0F, 5.0F);
        double settle = AutismHumanRotation.settleBandDegrees(0.0D);
        for (int i = 0; i < 500; i++) {
            Rotation out = AutismStealthAim.filter(state, config, random, goal, goal);
            double off = AutismRotationUtil.rotationAngleTo(goal, out);
            assertTrue(off <= 0.4D + settle + 1.0E-6D,
                "tick " + i + " wandered " + off + " degrees off goal");
            assertTrue(Float.isFinite(out.yaw()) && Float.isFinite(out.pitch()));
        }
    }

    @Test
    void stillTicksFreezeOnCurrentWhileSettled() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = quietConfig();
        config.pauses = true;
        config.holdChance = 1.0D;
        config.holdMaxTicks = 2;
        Random random = new Random(3L);
        Rotation goal = new Rotation(30.0F, 5.0F);
        Rotation out = AutismStealthAim.filter(state, config, random, goal, goal);
        assertEquals(goal.yaw(), out.yaw(), 1.0E-6F);
        assertEquals(goal.pitch(), out.pitch(), 1.0E-6F);
    }

    @Test
    void holdsDoNotFreezeATrackingTargetForever() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = quietConfig();
        config.pauses = true;
        config.holdChance = 1.0D;
        config.holdMaxTicks = 2;
        Random random = new Random(3L);
        Rotation current = new Rotation(0.0F, 0.0F);
        Rotation last = current;
        for (int i = 1; i <= 20; i++) {
            Rotation goal = new Rotation(i * 5.0F, 0.0F);
            last = AutismStealthAim.filter(state, config, random, goal, last);
        }
        assertTrue(wrapD(last.yaw() - current.yaw()) > 3.0D,
            "tracking stalled behind a moving goal, ended at " + last.yaw());
    }

    @Test
    void pursuitLagTrailsAFastStrafer() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = quietConfig();
        config.lag = true;
        config.lagGain = 0.35D;
        config.lagMaxDeg = 2.0D;
        Random random = new Random(11L);
        Rotation current = new Rotation(0.0F, 0.0F);
        Rotation last = null;
        Rotation lastGoal = null;
        for (int i = 1; i <= 30; i++) {
            Rotation goal = new Rotation(i * 4.0F, 0.0F);
            last = AutismStealthAim.filter(state, config, random, goal, current);
            lastGoal = goal;
            current = last;
        }
        double trail = wrapD(lastGoal.yaw() - last.yaw());
        assertTrue(trail > 0.2D && trail <= 2.0D + 1.0E-6D,
            "expected the published goal to trail the true goal, trailed by " + trail);
    }

    @Test
    void reversalKickOvershootsInTheOldDirection() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = quietConfig();
        config.overshoot = true;
        config.overshootDeg = 1.1D;
        Random random = new Random(21L);
        Rotation current = new Rotation(0.0F, 0.0F);
        for (int i = 1; i <= 8; i++) {
            Rotation goal = new Rotation(i * 6.0F, 0.0F);
            current = AutismStealthAim.filter(state, config, random, goal, current);
        }
        double maxLead = Double.NEGATIVE_INFINITY;
        for (int i = 1; i <= 6; i++) {
            Rotation goal = new Rotation(48.0F - i * 6.0F, 0.0F);
            Rotation out = AutismStealthAim.filter(state, config, random, goal, current);
            // Positive lead = published goal ahead in the OLD (+) direction: overshoot.
            maxLead = Math.max(maxLead, wrapD(out.yaw() - goal.yaw()));
            current = out;
        }
        assertTrue(maxLead > 0.3D,
            "no reversal overshoot observed after the strafe flip, max lead " + maxLead);
    }

    @Test
    void outputStaysFiniteAndPitchClamped() {
        AutismStealthAim.State state = new AutismStealthAim.State();
        AutismStealthAim.Config config = new AutismStealthAim.Config();
        Random random = new Random(99L);
        Rotation current = new Rotation(-170.0F, 80.0F);
        for (int i = 0; i < 1000; i++) {
            Rotation goal = new Rotation(
                (float) wrapD(-170.0D + i * 1.7D),
                89.0F - (i % 7));
            Rotation out = AutismStealthAim.filter(state, config, random, goal, current);
            assertTrue(Float.isFinite(out.yaw()) && Float.isFinite(out.pitch()),
                "tick " + i + " emitted non-finite rotation");
            assertTrue(Math.abs(out.pitch()) <= 89.9D + 1.0E-6D,
                "tick " + i + " escaped the pitch clamp: " + out.pitch());
            current = out;
        }
    }

    @Test
    void stealthProfileConvergesWithinCaps() {
        AutismHumanRotation.Stream s =
            new AutismHumanRotation.Stream(new Random(4L));
        AutismHumanRotation.seed(s, new Rotation(0.0F, 0.0F));
        Rotation goal = new Rotation(60.0F, -12.0F);
        Rotation prev = new Rotation(0.0F, 0.0F);
        Rotation emitted = prev;
        double gcd = 0.15D;
        for (int i = 0; i < 200; i++) {
            emitted = AutismHumanRotation.step(s, goal, 72.0F, 72.0F, gcd,
                true, AutismHumanRotation.MotionProfile.STEALTH);
            double dyaw = Math.abs(wrapD(emitted.yaw() - prev.yaw()));
            double dpitch = Math.abs(emitted.pitch() - prev.pitch());
            assertTrue(dyaw <= 72.0D + gcd + 1.0E-6D, "tick " + i + " yaw cap blown: " + dyaw);
            assertTrue(dpitch <= 72.0D + gcd + 1.0E-6D, "tick " + i + " pitch cap blown: " + dpitch);
            prev = emitted;
        }
        double settle = AutismHumanRotation.settleBandDegrees(gcd);
        // The stepper pursues its effective goal (true goal + the stream's
        // anti-fingerprint offset), so assert convergence there ...
        double[] effective = AutismHumanRotation.effectiveGoalForTesting(s);
        assertTrue(effective != null, "STEALTH run never rolled a stream");
        Rotation effGoal = new Rotation((float) effective[0], (float) effective[1]);
        assertTrue(AutismRotationUtil.rotationAngleTo(emitted, effGoal) <= settle + gcd,
            "STEALTH profile never settled, ended "
                + AutismRotationUtil.rotationAngleTo(emitted, effGoal) + " degrees out");
        // ... and bound the offset itself to its designed range (0.75-2.0 deg).
        assertTrue(AutismRotationUtil.rotationAngleTo(emitted, goal) <= 2.1D + settle + gcd,
            "STEALTH offset escaped its designed range");
    }
}
