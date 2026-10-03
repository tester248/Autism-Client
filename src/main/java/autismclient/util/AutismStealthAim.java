package autismclient.util;

import java.util.Objects;
import java.util.Random;

import autismclient.util.AutismRotationUtil.Rotation;

/**
 * Goal-space humanizer for combat aim ("stealth" aim).
 *
 * <p>Sits between target selection and {@link AutismKillAuraRotation#setTarget},
 * so every behavior below is expressed as a goal offset. The GCD-quantized
 * stepper in {@link AutismHumanRotation} still emits the final rotation, which
 * means sensitivity-grid and turn-cap guarantees are untouched.
 *
 * <p>Four behaviors, each independently toggleable:
 * <ul>
 *   <li><b>Wander</b> - a slow mean-reverting random walk around the true goal,
 *       like a rested hand that never sits perfectly still.</li>
 *   <li><b>Still ticks</b> - once converged, the goal occasionally freezes on
 *       the currently emitted rotation for 1-2 ticks. Real hands have frames
 *       with zero mouse delta; a stream that moves every tick does not.</li>
 *   <li><b>Pursuit lag</b> - the published goal trails the true goal
 *       proportionally to goal velocity, so fast strafers are chased instead
 *       of mirrored.</li>
 *   <li><b>Reversal overshoot</b> - when the goal's yaw/pitch velocity flips
 *       sign (a strafe reversal), a decaying impulse in the old direction is
 *       added, so the aim briefly overshoots before correcting.</li>
 * </ul>
 *
 * <p>Deliberately free of Minecraft imports so unit tests stay hermetic, just
 * like {@link AutismHumanRotation}.
 */
public final class AutismStealthAim {

    private static final double PITCH_LIMIT = 89.9D;

    private static final double VELOCITY_SMOOTHING = 0.5D;
    private static final double REVERSAL_MIN_YAW_SPEED = 1.5D;
    private static final double REVERSAL_MIN_PITCH_SPEED = 1.0D;
    private static final double OVERSHOOT_PITCH_SCALE = 0.6D;
    private static final double OVERSHOOT_DECAY = 0.5D;

    private static final double WANDER_STEP = 0.12D;
    private static final double WANDER_PULL = 0.92D;

    private static final int HOLD_COOLDOWN_TICKS = 6;

    private AutismStealthAim() {
    }

    /** Tunables, normally built from module settings once per tick. */
    public static final class Config {
        public boolean enabled = true;
        public double wanderMaxDeg = 0.4D;
        public boolean pauses = true;
        public double holdChance = 0.16D;
        public int holdMaxTicks = 2;
        public boolean lag = true;
        public double lagGain = 0.35D;
        public double lagMaxDeg = 2.0D;
        public boolean overshoot = true;
        public double overshootDeg = 1.1D;
    }

    /** Per-engagement memory. Reset on target loss, disengage, or module reset. */
    public static final class State {
        private double wanderYaw;
        private double wanderPitch;
        private int holdLeft;
        private int cooldown;
        private boolean hasLastGoal;
        private double lastGoalYaw;
        private double lastGoalPitch;
        private double velYaw;
        private double velPitch;
        private int lastDirYaw;
        private int lastDirPitch;
        private double overYaw;
        private double overPitch;

        public void reset() {
            wanderYaw = 0.0D;
            wanderPitch = 0.0D;
            holdLeft = 0;
            cooldown = 0;
            hasLastGoal = false;
            lastGoalYaw = 0.0D;
            lastGoalPitch = 0.0D;
            velYaw = 0.0D;
            velPitch = 0.0D;
            lastDirYaw = 0;
            lastDirPitch = 0;
            overYaw = 0.0D;
            overPitch = 0.0D;
        }
    }

    /**
     * Maps the true aim goal to the goal published to the rotation stream.
     * Returns {@code current} to hold perfectly still for this tick.
     *
     * @param state   engagement memory, mutated in place
     * @param config  tunables
     * @param random  randomness source
     * @param rawGoal true aim goal from target selection
     * @param current currently emitted rotation (hold anchor + convergence ref)
     */
    public static Rotation filter(State state, Config config, Random random,
                                  Rotation rawGoal, Rotation current) {
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(random, "random");
        Objects.requireNonNull(rawGoal, "rawGoal");
        Objects.requireNonNull(current, "current");
        if (config == null || !config.enabled) {
            return rawGoal;
        }

        double wanderMax = Math.max(0.0D, config.wanderMaxDeg);
        double holdChance = Math.min(1.0D, Math.max(0.0D, config.holdChance));
        int holdMax = Math.max(1, config.holdMaxTicks);
        double lagGain = Math.max(0.0D, config.lagGain);
        double lagMax = Math.max(0.0D, config.lagMaxDeg);
        double overshootDeg = Math.max(0.0D, config.overshootDeg);

        // Goal velocity (exponential moving average, degrees per tick).
        double dYaw = wrapDegrees(rawGoal.yaw() - state.lastGoalYaw);
        double dPitch = rawGoal.pitch() - state.lastGoalPitch;
        if (!state.hasLastGoal) {
            dYaw = 0.0D;
            dPitch = 0.0D;
            state.hasLastGoal = true;
        }
        state.velYaw += (dYaw - state.velYaw) * VELOCITY_SMOOTHING;
        state.velPitch += (dPitch - state.velPitch) * VELOCITY_SMOOTHING;
        state.lastGoalYaw = rawGoal.yaw();
        state.lastGoalPitch = rawGoal.pitch();

        // Strafe-reversal overshoot: kick in the old direction, then decay.
        // Direction memory only updates at meaningful speeds, so a slow
        // zero-crossing cannot rewrite history before the flip is confirmed.
        if (config.overshoot && overshootDeg > 0.0D) {
            if (Math.abs(state.velYaw) >= REVERSAL_MIN_YAW_SPEED) {
                int dirYaw = sign(state.velYaw);
                if (dirYaw != 0 && state.lastDirYaw != 0 && dirYaw != state.lastDirYaw) {
                    state.overYaw = overshootDeg * state.lastDirYaw;
                }
                if (dirYaw != 0) {
                    state.lastDirYaw = dirYaw;
                }
            }
            if (Math.abs(state.velPitch) >= REVERSAL_MIN_PITCH_SPEED) {
                int dirPitch = sign(state.velPitch);
                if (dirPitch != 0 && state.lastDirPitch != 0 && dirPitch != state.lastDirPitch) {
                    state.overPitch = overshootDeg * OVERSHOOT_PITCH_SCALE * state.lastDirPitch;
                }
                if (dirPitch != 0) {
                    state.lastDirPitch = dirPitch;
                }
            }
        }
        double appliedOverYaw = state.overYaw;
        double appliedOverPitch = state.overPitch;
        state.overYaw *= OVERSHOOT_DECAY;
        state.overPitch *= OVERSHOOT_DECAY;

        // Resting-hand wander: mean-reverting walk, always advancing.
        if (wanderMax > 0.0D) {
            state.wanderYaw = pull(state.wanderYaw
                + (random.nextDouble() - 0.5D) * WANDER_STEP, wanderMax);
            state.wanderPitch = pull(state.wanderPitch
                + (random.nextDouble() - 0.5D) * WANDER_STEP, wanderMax);
        } else {
            state.wanderYaw = 0.0D;
            state.wanderPitch = 0.0D;
        }

        boolean converged = AutismRotationUtil.rotationAngleTo(current, rawGoal)
            <= AutismHumanRotation.settleBandDegrees(0.0D);

        // Still ticks: freeze the goal on the emitted rotation while settled.
        if (config.pauses && holdChance > 0.0D) {
            if (state.holdLeft > 0) {
                state.holdLeft--;
                if (state.cooldown > 0) {
                    state.cooldown--;
                }
                return current;
            }
            if (converged && state.cooldown <= 0 && random.nextDouble() < holdChance) {
                state.holdLeft = Math.max(0, holdMax - 1);
                state.cooldown = HOLD_COOLDOWN_TICKS;
                return current;
            }
            if (state.cooldown > 0) {
                state.cooldown--;
            }
        }

        // Pursuit lag: trail fast-moving goals instead of mirroring them.
        double lagYaw = 0.0D;
        double lagPitch = 0.0D;
        if (config.lag && lagGain > 0.0D && lagMax > 0.0D) {
            lagYaw = clamp(-state.velYaw * lagGain, -lagMax, lagMax);
            lagPitch = clamp(-state.velPitch * lagGain, -lagMax, lagMax);
        }

        float yaw = (float) wrapDegrees(
            rawGoal.yaw() + state.wanderYaw + lagYaw + appliedOverYaw);
        float pitch = (float) clampPitch(
            rawGoal.pitch() + state.wanderPitch + lagPitch + appliedOverPitch);
        return new Rotation(yaw, pitch);
    }

    private static int sign(double value) {
        if (value > 0.0D) return 1;
        if (value < 0.0D) return -1;
        return 0;
    }

    private static double pull(double value, double max) {
        return clamp(value * WANDER_PULL, -max, max);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double clampPitch(double pitch) {
        return clamp(pitch, -PITCH_LIMIT, PITCH_LIMIT);
    }

    private static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0D;
        if (wrapped >= 180.0D) wrapped -= 360.0D;
        if (wrapped < -180.0D) wrapped += 360.0D;
        return wrapped;
    }
}
